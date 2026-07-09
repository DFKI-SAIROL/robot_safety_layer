#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <franka_custom_msgs/msg/fijk_debug.hpp>
#include <franka_custom_msgs/srv/set_pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
  void policyTargetDQCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetDQCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetPoseServiceCallback(const std::shared_ptr<franka_custom_msgs::srv::SetPoseStamped::Request> request,
                                 std::shared_ptr<franka_custom_msgs::srv::SetPoseStamped::Response> response);

  // ROS 2 components
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr safe_target_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Service<franka_custom_msgs::srv::SetPoseStamped>::SharedPtr target_pose_srv_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr other_joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_dq_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr policy_target_dq_sub_;

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
  Eigen::VectorXd target_dq_ik_;
  Eigen::VectorXd policy_target_dq_;
  bool is_policy_active_ = false;
  rclcpp::Time last_policy_msg_time_{ 0, 0, RCL_ROS_TIME };

  // Pinocchio
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex ee_frame_id_;

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

  arm_prefix_ = this->get_parameter("arm_prefix").as_string();
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

  target_pose_srv_ = this->create_service<franka_custom_msgs::srv::SetPoseStamped>(
      "target_pose",
      std::bind(&FrankaSafetyNode::targetPoseServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1, std::bind(&FrankaSafetyNode::jointStateCallback, this, std::placeholders::_1));

  target_dq_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "target_dq", 1, std::bind(&FrankaSafetyNode::targetDQCallback, this, std::placeholders::_1));

  policy_target_dq_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "policy_target_dq", 1, std::bind(&FrankaSafetyNode::policyTargetDQCallback, this, std::placeholders::_1));

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
  policy_target_dq_ = Eigen::VectorXd::Zero(model_.nv);

  // 6. Timer
  timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_dt_),
                                   std::bind(&FrankaSafetyNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Franka Safety Node initialized for %s.", arm_prefix_.c_str());
}

bool FrankaSafetyNode::loadPinocchioModel()
{
  if (!robot_safety_layer::ModelLoader::loadPinocchioModel(this, model_, data_))
    return false;
  std::string ee_name = arm_prefix_ + "fr3_link8";
  if (!model_.existFrame(ee_name))
    return false;
  ee_frame_id_ = model_.getFrameId(ee_name);
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

void FrankaSafetyNode::policyTargetDQCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 1. Ensure we have enough data (Franka needs at least 7 joints)
  if (msg->position.size() < 7)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Policy action too small! Got %zu, expected at least 7", msg->position.size());
    return;
  }

  // 2. Claim control and reset watchdog
  if (!is_policy_active_)
  {
    RCLCPP_INFO(this->get_logger(), "Engaging policy control.");
    is_policy_active_ = true;
  }
  last_policy_msg_time_ = this->get_clock()->now();
  // Policy/replay control must work even if teleop (targetDQCallback) is not used
  received_first_command_ = true;

  policy_target_dq_ = Eigen::VectorXd::Zero(model_.nv);

  // 3. MAP THE ARRAY TO THE COMMAND VECTOR
  if (msg->name.empty())
  {
    // Fallback: Python sent no names. Map by index directly to the first 7 joints.
    // (This assumes the network output perfectly matches the URDF joint order).
    for (int i = 0; i < 7; ++i)
    {
      policy_target_dq_[i] = msg->position[i];
    }
  }
  else
  {
    // Safe Mapping: Python sent names. Map them dynamically to Pinocchio's velocity vector.
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (model_.existJointName(msg->name[i]))
      {
        int idx_v = model_.joints[model_.getJointId(msg->name[i])].idx_v();
        if (idx_v >= 0 && idx_v < model_.nv)
        {
          policy_target_dq_[idx_v] = msg->position[i];
        }
      }
    }
  }

  // 4. Apply Velocity Limits Safely
  double max_dq = policy_target_dq_.array().abs().maxCoeff();

  // Ensure joint_velocity_limit_ is > 0 so we don't divide by zero or mute actions
  if (max_dq > joint_velocity_limit_ && joint_velocity_limit_ > 0.0)
  {
    policy_target_dq_ *= (joint_velocity_limit_ / max_dq);
  }
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

  // 3. Preempt policy control: a cartesian target from the user takes priority
  if (is_policy_active_)
  {
    RCLCPP_INFO(this->get_logger(), "Cartesian target received. Preempting policy control.");
    is_policy_active_ = false;
  }

  response->success = true;
  response->message = "Safe target pose forwarded to IK node.";
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
    return;
  }

  if (q_cmd_.size() == 0)
  {
    q_cmd_ = q_;
  }

  auto now = this->get_clock()->now();

  // Arbiter
  Eigen::VectorXd active_dq;
  if (is_policy_active_)
  {
    if ((now - last_policy_msg_time_).seconds() > 0.25)
    {
      RCLCPP_WARN(this->get_logger(), "Policy timeout. Reverting to IK.");
      is_policy_active_ = false;
      active_dq = target_dq_ik_;
    }
    else
    {
      active_dq = policy_target_dq_;
    }
  }
  else
  {
    active_dq = target_dq_ik_;
  }

  // Safety
  if (!bypass_safety_)
  {
    pinocchio::forwardKinematics(model_, *data_, q_cmd_);
    pinocchio::updateFramePlacements(model_, *data_);
    Eigen::Vector3d current_pos = data_->oMf[ee_frame_id_].translation();

    Eigen::MatrixXd J(6, model_.nv);
    J.setZero();
    pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
    Eigen::VectorXd cart_vel = J * active_dq;

    double max_v = safety_layer_.getMaxSafeVelocity(current_pos, cart_vel.head<3>());
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
