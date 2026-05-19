#pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace robot_safety_layer {

/**
 * @brief Utility class to load Pinocchio models from ROS 2 parameters.
 */
class ModelLoader {
public:
    /**
     * @brief Fetches URDF from a parameter and builds a Pinocchio model.
     */
    static bool loadPinocchioModel(
        rclcpp::Node* node, 
        pinocchio::Model& model, 
        std::unique_ptr<pinocchio::Data>& data,
        const std::string& remote_node_name = "robot_state_publisher",
        const std::string& parameter_name = "robot_description"
    );

    /**
     * @brief Specialized loader for a second robot (using namespace-based robot_state_publisher).
     */
    static bool loadOtherPinocchioModel(
        rclcpp::Node* node, 
        pinocchio::Model& model, 
        std::unique_ptr<pinocchio::Data>& data,
        const std::string& other_ns
    );
};

} // namespace robot_safety_layer
