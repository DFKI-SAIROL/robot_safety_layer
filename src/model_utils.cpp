#include "model_utils.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot_safety_layer {

bool ModelLoader::loadPinocchioModel(
    rclcpp::Node* node, 
    pinocchio::Model& model, 
    std::unique_ptr<pinocchio::Data>& data,
    const std::string& remote_node_name,
    const std::string& parameter_name
) {
    using namespace std::chrono_literals;

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node_name);

    RCLCPP_INFO(node->get_logger(), "Waiting for '%s' parameter server...", remote_node_name.c_str());
    while (!param_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) return false;
    }

    while (!param_client->has_parameter(parameter_name)) {
        RCLCPP_INFO(node->get_logger(), "Waiting for '%s' parameter...", parameter_name.c_str());
        rclcpp::sleep_for(500ms);
    }

    std::string urdf_string = param_client->get_parameter<std::string>(parameter_name);
    if (urdf_string.empty()) return false;

    try {
        pinocchio::urdf::buildModelFromXML(urdf_string, model);
        data = std::make_unique<pinocchio::Data>(model);
    } catch (...) {
        return false;
    }

    return true;
}

bool ModelLoader::loadOtherPinocchioModel(
    rclcpp::Node* node, 
    pinocchio::Model& model, 
    std::unique_ptr<pinocchio::Data>& data,
    const std::string& other_ns
) {
    std::string remote_node = "/" + other_ns + "/robot_state_publisher";
    return loadPinocchioModel(node, model, data, remote_node, "robot_description");
}

} // namespace robot_safety_layer


