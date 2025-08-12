#include "jetcobot_moveit_picker/gripper_controller.hpp"

namespace jetcobot_picker {

GripperController::GripperController(rclcpp::Node::SharedPtr node) : node_(node) {
    gripper_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/gripper_command", 10);
    RCLCPP_INFO(node_->get_logger(), "GripperController initialized");
}

void GripperController::controlGripper(int close_value) {
    auto gripper_msg = std_msgs::msg::Int32();
    gripper_msg.data = close_value;
    gripper_pub_->publish(gripper_msg);
    
    if (close_value == GripperPositions::FULLY_OPEN) {
        RCLCPP_INFO(node_->get_logger(), "Opening gripper fully...");
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    } else if (close_value == GripperPositions::FULLY_CLOSED) {
        RCLCPP_INFO(node_->get_logger(), "Closing gripper fully...");
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::GRIPPER_CLOSE_DELAY_MS));
    } else {
        RCLCPP_INFO(node_->get_logger(), "Setting gripper position to %d", close_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    }
}

void GripperController::publishGripperCommand(int value, const std::string& description, int delay_ms) {
    auto gripper_msg = std_msgs::msg::Int32();
    gripper_msg.data = value;
    gripper_pub_->publish(gripper_msg);
    
    RCLCPP_INFO(node_->get_logger(), "%s (value: %d)", description.c_str(), value);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

void GripperController::openGripperFully() {
    publishGripperCommand(GripperPositions::FULLY_OPEN, "Opening gripper fully", TimingConstants::OPERATION_DELAY_MS);
}

void GripperController::closeGripperFully() {
    publishGripperCommand(GripperPositions::FULLY_CLOSED, "Closing gripper fully", TimingConstants::GRIPPER_CLOSE_DELAY_MS);
}

void GripperController::openGripperToHoldingPosition() {
    publishGripperCommand(GripperPositions::HOLDING_POSITION, "Opening gripper to holding position", TimingConstants::OPERATION_DELAY_MS);
}

void GripperController::closeGripperToPicking() {
    publishGripperCommand(GripperPositions::PICKING_POSITION, "Closing gripper to picking position", TimingConstants::GRIPPER_CLOSE_DELAY_MS);
}

} // namespace jetcobot_picker
