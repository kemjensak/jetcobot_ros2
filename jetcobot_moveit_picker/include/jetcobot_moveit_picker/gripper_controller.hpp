#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "constants.hpp"
#include <chrono>
#include <thread>

namespace jetcobot_picker {

class GripperController {
public:
    explicit GripperController(rclcpp::Node::SharedPtr node);
    
    void controlGripper(int close_value);
    void openGripperFully();
    void closeGripperFully();
    void openGripperToHoldingPosition();
    void closeGripperToPicking();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;
    
    void publishGripperCommand(int value, const std::string& description, int delay_ms);
};

} // namespace jetcobot_picker
