#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <franka_custom_msgs/msg/fijk_debug.hpp>
#include <franka_custom_msgs/srv/set_pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "safety_layer.hpp"
#include "model_utils.hpp"

using namespace std::chrono_literals;

class FrankaSafetyNode : public rclcpp::Node
{
public:
  FrankaSafetyNode();

private:
  // Core logic
  bool loadPinocchioModel();
  bool loadOtherPinocchioModel(const std::string& other_ns);
  void controlLoop();

  // Callbacks
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void otherJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void jointVelocityCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void jointPositionCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetDQCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  Eigen::VectorXd jointPositionDQ(const rclcpp::Time& now);
  void handOverToIK();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);
  void publishDebugInfo(const rclcpp::Time& stamp, const Eigen::VectorXd& active_dq,
                        const Eigen::VectorXd& cartesian_velocity);
  geometry_msgs::msg::Pose toPoseMsg(const pinocchio::SE3& se3) const;
  void targetPoseServiceCallback(const std::shared_ptr<franka_custom_msgs::srv::SetPoseStamped::Request> request,
                                 std::shared_ptr<franka_custom_msgs::srv::SetPoseStamped::Response> response);

  // ROS 2 components
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr safe_target_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<franka_custom_msgs::msg::FIJKDebug>::SharedPtr debug_pub_;

  rclcpp::Service<franka_custom_msgs::srv::SetPoseStamped>::SharedPtr target_pose_srv_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr other_joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_dq_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_velocity_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_position_cmd_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters & State
  std::string arm_prefix_;
  std::string other_ns_;
  double joint_velocity_limit_ = 1.0;
  double max_spring_stretch_ = 0.1;
  double timer_dt_ = 0.005;
  bool bypass_safety_ = false;

  Eigen::VectorXd q_;
  Eigen::VectorXd q_cmd_;
  bool received_first_command_ = false;
  // What is currently driving q_cmd_. One value, so IK and the two external command
  // channels cannot be active at once.
  enum class CommandSource
  {
    IK,
    JOINT_VELOCITY,
    JOINT_POSITION,
  };
  CommandSource command_source_ = CommandSource::IK;
  rclcpp::Time last_external_cmd_time_{ 0, 0, RCL_ROS_TIME };

  Eigen::VectorXd target_dq_ik_;
  Eigen::VectorXd cmd_joint_velocity_;

  // Last poses seen by the target_pose service, replayed into fijk_debug.
  geometry_msgs::msg::Pose last_target_pose_;
  geometry_msgs::msg::Pose last_safe_target_pose_;

  // Joint-position commands are ramped to over one target period from where q_cmd_ was
  // when they arrived; targets further than cmd_joint_max_deviation_ from measured are
  // rejected.
  Eigen::VectorXd cmd_joint_start_;
  Eigen::VectorXd cmd_joint_target_;
  rclcpp::Time cmd_joint_target_time_{ 0, 0, RCL_ROS_TIME };
  double cmd_joint_target_dt_ = 1.0 / 30.0;
  double cmd_joint_max_deviation_ = 0.3;
  // While joint commands drive, the IK node keeps running on its last Cartesian target;
  // re-target it at the measured pose this often so handing control back is seamless.
  double ik_sync_period_ = 0.02;
  rclcpp::Time last_ik_sync_time_{ 0, 0, RCL_ROS_TIME };

  // Pinocchio
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex ee_frame_id_;
  std::string end_effector_frame_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  SafetyLayer safety_layer_;
};

FrankaSafetyNode::FrankaSafetyNode() : Node("franka_controller_node")
{
  // 1. Parameters
  this->declare_parameter("arm_prefix", "");
  this->declare_parameter("joint_velocity_limit", 1.0);
  this->declare_parameter("max_spring_stretch", 0.1);
  this->declare_parameter("timer_dt", 0.005);
  this->declare_parameter("bypass_safety", false);
  this->declare_parameter("end_effector_frame", "");
  this->declare_parameter("cmd_joint_target_dt", 1.0 / 30.0);
  this->declare_parameter("cmd_joint_max_deviation", 0.3);

  arm_prefix_ = this->get_parameter("arm_prefix").as_string();
  end_effector_frame_ = this->get_parameter("end_effector_frame").as_string();
  cmd_joint_target_dt_ = this->get_parameter("cmd_joint_target_dt").as_double();
  cmd_joint_max_deviation_ = this->get_parameter("cmd_joint_max_deviation").as_double();

  // Live-tunable so teleop and policy rates can be switched without restarting the
  // stack (DAgger alternates between them). Only values that are read fresh each
  // control cycle are here; timer_dt would need the timer rebuilt, and bypass_safety
  // is deliberately left static.
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FrankaSafetyNode::parametersCallback, this, std::placeholders::_1));
  if (!arm_prefix_.empty() && arm_prefix_.back() != '_')
    arm_prefix_ += "_";

  joint_velocity_limit_ = this->get_parameter("joint_velocity_limit").as_double();
  max_spring_stretch_ = this->get_parameter("max_spring_stretch").as_double();
  timer_dt_ = this->get_parameter("timer_dt").as_double();
  bypass_safety_ = this->get_parameter("bypass_safety").as_bool();

  other_ns_ = (arm_prefix_ == "franka_left_") ? "franka_right" : "franka_left";

  // Load Pinocchio Model
  if (!loadPinocchioModel())
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to load Pinocchio model. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  // Load Pinocchio Model for the other robot
  if (safety_layer_.other_robot_check && !bypass_safety_)
  {
    if (!loadOtherPinocchioModel(other_ns_))
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to load other Pinocchio model. Shutting down.");
      rclcpp::shutdown();
      return;
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Not loading other Pinocchio model (bypass_safety is true).");
  }

  // Pub/Sub
  target_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("target_joint", 1);
  safe_target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("safe_target_pose", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("safety_vis", 10);
  debug_pub_ = this->create_publisher<franka_custom_msgs::msg::FIJKDebug>("fijk_debug", 1);

  target_pose_srv_ = this->create_service<franka_custom_msgs::srv::SetPoseStamped>(
      "target_pose",
      std::bind(&FrankaSafetyNode::targetPoseServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1, std::bind(&FrankaSafetyNode::jointStateCallback, this, std::placeholders::_1));

  target_dq_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "target_dq", 1, std::bind(&FrankaSafetyNode::targetDQCallback, this, std::placeholders::_1));

  joint_velocity_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "command/joint_velocity", 1,
      std::bind(&FrankaSafetyNode::jointVelocityCommandCallback, this, std::placeholders::_1));

  joint_position_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "command/joint_position", 1,
      std::bind(&FrankaSafetyNode::jointPositionCommandCallback, this, std::placeholders::_1));

  other_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/" + other_ns_ + "/joint_states", 1,
      std::bind(&FrankaSafetyNode::otherJointStateCallback, this, std::placeholders::_1));

  if (!bypass_safety_)
  {
    // Parameters for Initial Position
    this->declare_parameter("init_joint_position", std::vector<double>(7, 0.0));

    std::vector<double> init_joint_position_vec = this->get_parameter("init_joint_position").as_double_array();
    if (init_joint_position_vec.size() != 7 ||
        std::all_of(init_joint_position_vec.begin(), init_joint_position_vec.end(),
                    [](double x) { return std::abs(x) < 1e-6; }))
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid init joint positions (wrong size or all zeros). Shutting down.");
      rclcpp::shutdown();
      return;
    }
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(model_.nv);
    for (int i = 0; i < 7; ++i)
    {
      std::string name = arm_prefix_ + "fr3_joint" + std::to_string(i + 1);
      if (model_.existJointName(name))
      {
        int idx = model_.joints[model_.getJointId(name)].idx_q();
        q_init(idx) = init_joint_position_vec[i];
      }
    }
    pinocchio::forwardKinematics(model_, *data_, q_init);
    pinocchio::updateFramePlacements(model_, *data_);
    Eigen::Vector3d init_cartesian = data_->oMf[ee_frame_id_].translation();
    safety_layer_.init(init_cartesian, marker_pub_);
  }

  // 5. Initialize State Vectors
  q_ = Eigen::VectorXd::Zero(model_.nv);
  // q_cmd_ intentionally left as size-0 — initialized from real q_ on first command
  target_dq_ik_ = Eigen::VectorXd::Zero(model_.nv);
  cmd_joint_velocity_ = Eigen::VectorXd::Zero(model_.nv);

  // 6. Timer
  timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_dt_),
                                   std::bind(&FrankaSafetyNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Franka Safety Node initialized for %s.", arm_prefix_.c_str());
}

bool FrankaSafetyNode::loadPinocchioModel()
{
  if (!robot_safety_layer::ModelLoader::loadPinocchioModel(this, model_, data_))
    return false;
  // Must match the frame the IK layer controls, or safety is evaluated at a different
  // point on the arm than the one being commanded.
  if (end_effector_frame_.empty())
  {
    end_effector_frame_ = arm_prefix_ + "grasp_point";
    RCLCPP_WARN(this->get_logger(), "end_effector_frame parameter not set. Falling back to default: %s",
                end_effector_frame_.c_str());
  }
  if (!model_.existFrame(end_effector_frame_))
  {
    RCLCPP_ERROR(this->get_logger(), "End effector frame '%s' not found in model.",
                 end_effector_frame_.c_str());
    return false;
  }
  ee_frame_id_ = model_.getFrameId(end_effector_frame_);
  return true;
}

bool FrankaSafetyNode::loadOtherPinocchioModel(const std::string& other_ns)
{
  if (!robot_safety_layer::ModelLoader::loadOtherPinocchioModel(this, safety_layer_.other_model_,
                                                                safety_layer_.other_data_, other_ns))
  {
    return false;
  }

  // Identify other end effector frame
  std::string other_ee_name = other_ns + "_fr3_link8";
  if (!safety_layer_.other_model_.existFrame(other_ee_name))
  {
    RCLCPP_ERROR(this->get_logger(), "Other EE frame %s not found in other model.", other_ee_name.c_str());
    return false;
  }
  safety_layer_.other_ee_frame_id_ = safety_layer_.other_model_.getFrameId(other_ee_name);

  safety_layer_.other_robot_check = true;
  return true;
}

void FrankaSafetyNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (q_.size() != model_.nv)
    q_ = Eigen::VectorXd::Zero(model_.nv);
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (model_.existJointName(msg->name[i]))
    {
      int idx = model_.joints[model_.getJointId(msg->name[i])].idx_q();
      q_(idx) = msg->position[i];
    }
  }
}

void FrankaSafetyNode::otherJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (safety_layer_.other_q_.size() != safety_layer_.other_model_.nv)
  {
    safety_layer_.other_q_ = Eigen::VectorXd::Zero(safety_layer_.other_model_.nv);
  }
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (safety_layer_.other_model_.existJointName(msg->name[i]))
    {
      int idx = safety_layer_.other_model_.joints[safety_layer_.other_model_.getJointId(msg->name[i])].idx_q();
      safety_layer_.other_q_(idx) = msg->position[i];
    }
  }
}

void FrankaSafetyNode::jointVelocityCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->velocity.empty())
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "command/joint_velocity: velocity field is empty%s. Joint velocities must be "
                          "sent in JointState.velocity; ignored.",
                          msg->position.empty() ? "" : " (positions were sent instead)");
    return;
  }
  if (msg->velocity.size() < 7)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "command/joint_velocity: got %zu velocities, expected at least 7; ignored.",
                         msg->velocity.size());
    return;
  }

  if (command_source_ != CommandSource::JOINT_VELOCITY)
    RCLCPP_INFO(this->get_logger(), "Engaging joint-velocity command control.");

  command_source_ = CommandSource::JOINT_VELOCITY;
  last_external_cmd_time_ = this->get_clock()->now();
  // Works even if teleop (targetDQCallback) never ran.
  received_first_command_ = true;

  cmd_joint_velocity_ = Eigen::VectorXd::Zero(model_.nv);
  if (msg->name.empty())
  {
    // No names: map by index to the first 7 joints, assuming the sender matches URDF order.
    for (int i = 0; i < 7; ++i)
      cmd_joint_velocity_[i] = msg->velocity[i];
  }
  else
  {
    for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i)
    {
      if (!model_.existJointName(msg->name[i]))
        continue;
      const int idx_v = model_.joints[model_.getJointId(msg->name[i])].idx_v();
      if (idx_v >= 0 && idx_v < model_.nv)
        cmd_joint_velocity_[idx_v] = msg->velocity[i];
    }
  }

  const double max_dq = cmd_joint_velocity_.array().abs().maxCoeff();
  if (max_dq > joint_velocity_limit_ && joint_velocity_limit_ > 0.0)
    cmd_joint_velocity_ *= (joint_velocity_limit_ / max_dq);
}

void FrankaSafetyNode::jointPositionCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Absolute joint-position targets. Names are mandatory: mapping by index would
  // silently depend on the sender's ordering.
  if (!q_.allFinite() || msg->name.size() != msg->position.size())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "command/joint_position: no joint state yet, or name/position size mismatch; ignored.");
    return;
  }

  Eigen::VectorXd target = (q_cmd_.size() == model_.nv) ? q_cmd_ : q_;
  for (int i = 0; i < 7; ++i)
  {
    const std::string name = arm_prefix_ + "fr3_joint" + std::to_string(i + 1);
    const auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "command/joint_position: missing joint %s; ignored.", name.c_str());
      return;
    }
    const double value = msg->position[std::distance(msg->name.begin(), it)];
    const int idx = model_.joints[model_.getJointId(name)].idx_q();
    if (!std::isfinite(value) || std::abs(value - q_(idx)) > cmd_joint_max_deviation_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "command/joint_position: %s target %.3f is %.3f rad from the measured %.3f "
                           "(limit %.2f); ignored.",
                           name.c_str(), value, std::abs(value - q_(idx)), q_(idx), cmd_joint_max_deviation_);
      return;
    }
    target(idx) = value;
  }

  if (q_cmd_.size() != model_.nv)
    q_cmd_ = q_;
  if (command_source_ != CommandSource::JOINT_POSITION)
    RCLCPP_INFO(this->get_logger(), "Engaging joint-position command control.");

  received_first_command_ = true;
  command_source_ = CommandSource::JOINT_POSITION;
  last_external_cmd_time_ = this->get_clock()->now();
  cmd_joint_start_ = q_cmd_;
  cmd_joint_target_ = target;
  cmd_joint_target_time_ = last_external_cmd_time_;
}

Eigen::VectorXd FrankaSafetyNode::jointPositionDQ(const rclcpp::Time& now)
{
  // Ramp linearly from where q_cmd_ was at arrival to the target over one target period,
  // then hold. An early target shortens the ramp, a late one leaves q_cmd_ holding.
  const double alpha = std::clamp((now - cmd_joint_target_time_).seconds() / cmd_joint_target_dt_, 0.0, 1.0);
  const Eigen::VectorXd q_ref = cmd_joint_start_ + alpha * (cmd_joint_target_ - cmd_joint_start_);

  Eigen::VectorXd dq = (q_ref - q_cmd_) / timer_dt_;
  const double max_dq = dq.array().abs().maxCoeff();
  if (max_dq > joint_velocity_limit_ && joint_velocity_limit_ > 0.0)
    dq *= joint_velocity_limit_ / max_dq;
  return dq;
}

void FrankaSafetyNode::handOverToIK()
{
  // The IK node still holds the pose target it had before the joint command took over and
  // would drive back to it. Re-target it at the current pose instead.
  pinocchio::forwardKinematics(model_, *data_, q_);
  pinocchio::updateFramePlacements(model_, *data_);
  const pinocchio::SE3& pose = data_->oMf[ee_frame_id_];

  geometry_msgs::msg::PoseStamped hold;
  hold.header.stamp = this->get_clock()->now();
  hold.header.frame_id = "world";
  hold.pose.position.x = pose.translation()(0);
  hold.pose.position.y = pose.translation()(1);
  hold.pose.position.z = pose.translation()(2);
  const Eigen::Quaterniond q(pose.rotation());
  hold.pose.orientation.x = q.x();
  hold.pose.orientation.y = q.y();
  hold.pose.orientation.z = q.z();
  hold.pose.orientation.w = q.w();
  safe_target_pose_pub_->publish(hold);

  // Its last dq was computed against the old target; drop it until a fresh one arrives.
  target_dq_ik_.setZero();
}

void FrankaSafetyNode::targetDQCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (target_dq_ik_.size() != model_.nv)
    target_dq_ik_ = Eigen::VectorXd::Zero(model_.nv);
  received_first_command_ = true;

  if (msg->name.empty())
  {
    for (int i = 0; i < std::min((int)msg->velocity.size(), 7); ++i)
      target_dq_ik_[i] = msg->velocity[i];
  }
  else
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (model_.existJointName(msg->name[i]))
      {
        int idx = model_.joints[model_.getJointId(msg->name[i])].idx_v();
        if (idx >= 0 && idx < model_.nv)
          target_dq_ik_(idx) = msg->velocity[i];
      }
    }
  }
}

void FrankaSafetyNode::targetPoseServiceCallback(
    const std::shared_ptr<franka_custom_msgs::srv::SetPoseStamped::Request> request,
    std::shared_ptr<franka_custom_msgs::srv::SetPoseStamped::Response> response)
{
  // 1. Apply proactive safety: slide the desired pose away from obstacles
  geometry_msgs::msg::PoseStamped safe_msg = request->pose;
  if (!bypass_safety_ && q_cmd_.size() == model_.nv)
  {
    const auto& p = request->pose.pose;
    Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
    Eigen::Quaterniond quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    pinocchio::SE3 desired_pose(quat.toRotationMatrix(), t);

    pinocchio::forwardKinematics(model_, *data_, q_cmd_);
    pinocchio::updateFramePlacements(model_, *data_);
    pinocchio::SE3 robot_pose = data_->oMf[ee_frame_id_];

    pinocchio::SE3 safe_pose = safety_layer_.adjustToSafePose(robot_pose, desired_pose);

    safe_msg.pose.position.x = safe_pose.translation().x();
    safe_msg.pose.position.y = safe_pose.translation().y();
    safe_msg.pose.position.z = safe_pose.translation().z();
    Eigen::Quaterniond safe_quat(safe_pose.rotation());
    safe_msg.pose.orientation.w = safe_quat.w();
    safe_msg.pose.orientation.x = safe_quat.x();
    safe_msg.pose.orientation.y = safe_quat.y();
    safe_msg.pose.orientation.z = safe_quat.z();
  }

  // 2. Forward safe pose to IK Node
  safe_target_pose_pub_->publish(safe_msg);

  last_target_pose_ = request->pose.pose;
  last_safe_target_pose_ = safe_msg.pose;

  // 3. A Cartesian target from the user takes priority over any joint command.
  // No hand-over needed: this call just gave the IK node a fresh target.
  if (command_source_ != CommandSource::IK)
  {
    RCLCPP_INFO(this->get_logger(), "Cartesian target received. Preempting joint command control.");
    command_source_ = CommandSource::IK;
  }

  response->success = true;
  response->message = "Safe target pose forwarded to IK node.";
}

geometry_msgs::msg::Pose FrankaSafetyNode::toPoseMsg(const pinocchio::SE3& se3) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = se3.translation()(0);
  pose.position.y = se3.translation()(1);
  pose.position.z = se3.translation()(2);
  const Eigen::Quaterniond q(se3.rotation());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

void FrankaSafetyNode::publishDebugInfo(const rclcpp::Time& stamp, const Eigen::VectorXd& active_dq,
                                        const Eigen::VectorXd& cartesian_velocity)
{
  franka_custom_msgs::msg::FIJKDebug msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "world";

  // actual_pose is the measured arm, cmd_pose the configuration being commanded. They differ
  // by the impedance sag, which is why consumers must not anchor on cmd_pose.
  pinocchio::forwardKinematics(model_, *data_, q_);
  pinocchio::updateFramePlacements(model_, *data_);
  msg.actual_pose = toPoseMsg(data_->oMf[ee_frame_id_]);

  if (q_cmd_.size() == model_.nq)
  {
    pinocchio::forwardKinematics(model_, *data_, q_cmd_);
    pinocchio::updateFramePlacements(model_, *data_);
    msg.cmd_pose = toPoseMsg(data_->oMf[ee_frame_id_]);
  }

  msg.target_pose = last_target_pose_;
  msg.safe_target_pose = last_safe_target_pose_;

  msg.safety_distance = static_cast<float>(safety_layer_.current_distance_to_obstacle);
  msg.safety_distance_along_velocity =
      static_cast<float>(safety_layer_.current_distance_to_obstacle_along_velocity_direction);

  msg.cartesian_velocity.linear.x = cartesian_velocity(0);
  msg.cartesian_velocity.linear.y = cartesian_velocity(1);
  msg.cartesian_velocity.linear.z = cartesian_velocity(2);
  msg.cartesian_velocity.angular.x = cartesian_velocity(3);
  msg.cartesian_velocity.angular.y = cartesian_velocity(4);
  msg.cartesian_velocity.angular.z = cartesian_velocity(5);

  msg.q_actual.resize(7);
  msg.q_target.resize(7);
  msg.dq_commanded.resize(7);
  for (int i = 0; i < 7; ++i)
  {
    const std::string name = arm_prefix_ + "fr3_joint" + std::to_string(i + 1);
    const auto joint_id = model_.getJointId(name);
    const int idx_q = model_.joints[joint_id].idx_q();
    const int idx_v = model_.joints[joint_id].idx_v();
    msg.q_actual[i] = q_(idx_q);
    msg.q_target[i] = (q_cmd_.size() == model_.nq) ? q_cmd_(idx_q) : q_(idx_q);
    msg.dq_commanded[i] = (active_dq.size() == model_.nv) ? active_dq(idx_v) : 0.0;
  }

  debug_pub_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult FrankaSafetyNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : parameters)
  {
    const std::string& name = param.get_name();
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      continue;
    const double value = param.as_double();

    if (name == "max_spring_stretch" || name == "joint_velocity_limit" ||
        name == "cmd_joint_target_dt" || name == "cmd_joint_max_deviation")
    {
      if (value <= 0.0)
      {
        result.successful = false;
        result.reason = name + " must be > 0";
        return result;
      }
    }

    if (name == "max_spring_stretch")
      max_spring_stretch_ = value;
    else if (name == "joint_velocity_limit")
      joint_velocity_limit_ = value;
    else if (name == "cmd_joint_target_dt")
      cmd_joint_target_dt_ = value;
    else if (name == "cmd_joint_max_deviation")
      cmd_joint_max_deviation_ = value;
    else
      continue;

    RCLCPP_INFO(this->get_logger(), "Updated %s to %f", name.c_str(), value);
  }
  return result;
}

void FrankaSafetyNode::controlLoop()
{
  if (!q_.allFinite())
  {
    return;
  }
  if (!received_first_command_)
  {
    q_cmd_ = q_;
    // Command equals measured and nothing is moving, but clients that bootstrap their state
    // from fijk_debug need it before the first command arrives.
    publishDebugInfo(this->get_clock()->now(), Eigen::VectorXd::Zero(model_.nv),
                     Eigen::VectorXd::Zero(6));
    return;
  }

  if (q_cmd_.size() == 0)
  {
    q_cmd_ = q_;
  }

  auto now = this->get_clock()->now();

  // Arbiter
  Eigen::VectorXd active_dq;
  if (command_source_ != CommandSource::IK && (now - last_external_cmd_time_).seconds() > 0.25)
  {
    RCLCPP_WARN(this->get_logger(), "Joint command timeout. Reverting to IK.");
    const bool was_position = command_source_ == CommandSource::JOINT_POSITION;
    command_source_ = CommandSource::IK;
    if (was_position)
      handOverToIK();
  }

  switch (command_source_)
  {
    case CommandSource::JOINT_VELOCITY:
      active_dq = cmd_joint_velocity_;
      break;
    case CommandSource::JOINT_POSITION:
      active_dq = jointPositionDQ(now);
      // Keep the idle IK node on the current pose so it is already current when control
      // hands back, instead of pulling toward the target it had before.
      if ((now - last_ik_sync_time_).seconds() >= ik_sync_period_)
      {
        handOverToIK();
        last_ik_sync_time_ = now;
      }
      break;
    case CommandSource::IK:
      active_dq = target_dq_ik_;
      break;
  }

  // Commanded kinematics. Computed unconditionally: the safety branch needs them only when
  // enabled, but fijk_debug reports them either way, and it must report the velocity that is
  // actually sent - so the Jacobian is reused below, after any safety clamp.
  pinocchio::forwardKinematics(model_, *data_, q_cmd_);
  pinocchio::updateFramePlacements(model_, *data_);
  const Eigen::Vector3d current_pos = data_->oMf[ee_frame_id_].translation();

  Eigen::MatrixXd J(6, model_.nv);
  J.setZero();
  pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);

  // Safety
  if (!bypass_safety_)
  {
    const Eigen::VectorXd cart_vel = J * active_dq;
    const double max_v = safety_layer_.getMaxSafeVelocity(current_pos, cart_vel.head<3>());
    if (cart_vel.head<3>().norm() > max_v && cart_vel.head<3>().norm() > 1e-6)
    {
      active_dq *= (max_v / cart_vel.head<3>().norm());
    }
  }

  // Integration
  sensor_msgs::msg::JointState cmd;
  cmd.header.stamp = now;
  cmd.name.resize(7);
  cmd.position.resize(7);
  cmd.velocity.resize(7);

  for (int i = 0; i < 7; ++i)
  {
    std::string name = arm_prefix_ + "fr3_joint" + std::to_string(i + 1);
    cmd.name[i] = name;
    int idx = model_.joints[model_.getJointId(name)].idx_q();
    q_cmd_(idx) += active_dq(idx) * timer_dt_;

    double error = q_cmd_(idx) - q_(idx);
    if (error > max_spring_stretch_)
      q_cmd_(idx) = q_(idx) + max_spring_stretch_;
    else if (error < -max_spring_stretch_)
      q_cmd_(idx) = q_(idx) - max_spring_stretch_;

    cmd.position[i] = q_cmd_(idx);
    cmd.velocity[i] = active_dq(idx);
  }

  target_joint_pub_->publish(cmd);
  publishDebugInfo(now, active_dq, J * active_dq);
  if (!bypass_safety_)
    safety_layer_.vis_.publish_markers();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrankaSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
