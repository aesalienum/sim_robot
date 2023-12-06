#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hardware/diffbot.h"

void declare_parameters(rclcpp::Node::SharedPtr node) {
    // diffbot
    node->declare_parameter<std::string>("joint_name_left_wheel", "left_wheel_joint");
    node->declare_parameter<std::string>("joint_name_right_wheel", "right_wheel_joint");
    node->declare_parameter<double>("wheel_radius", 0.07);
    node->declare_parameter<double>("distance_to_left_wheel", 0.173205);
    node->declare_parameter<double>("distance_to_right_wheel", 0.173205);

    // common
    node->declare_parameter<std::string>("topic_name_joint_states", "joint_states");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto hardware_node = std::make_shared<rclcpp::Node>("hardware_node");

    auto device_diffbot = hardware::DeviceDiffBot(hardware_node);

    rclcpp::spin(hardware_node);
    rclcpp::shutdown();
    return 0;
}