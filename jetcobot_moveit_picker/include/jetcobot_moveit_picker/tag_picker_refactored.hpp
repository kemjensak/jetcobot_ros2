#pragma once

#include "gripper_controller.hpp"
#include "tag_detection_manager.hpp"
#include "motion_planner.hpp"
#include "pick_place_executor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <jetcobot_interfaces/action/picker_action.hpp>
#include <memory>

namespace jetcobot_picker {

using PickerAction = jetcobot_interfaces::action::PickerAction;
using GoalHandlePickerAction = rclcpp_action::ServerGoalHandle<PickerAction>;

class TagPicker : public rclcpp::Node {
public:
    TagPicker();
    bool execute();

private:
    // Action server components
    rclcpp_action::Server<PickerAction>::SharedPtr action_server_;
    
    // Component managers
    std::shared_ptr<GripperController> gripper_controller_;
    std::shared_ptr<TagDetectionManager> tag_detection_manager_;
    std::shared_ptr<MotionPlanner> motion_planner_;
    std::shared_ptr<PickPlaceExecutor> pick_place_executor_;
    
    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickerAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    void execute_goal(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    
    // Command handlers
    bool handleHomeCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    bool handleScanCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    bool handleScanFrontCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    bool handleScanLeftCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    bool handleScanRightCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    bool handlePickAndPlaceCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    
    // Helper methods
    bool executeScanWithConfiguration(const std::string& config_name, const std::shared_ptr<GoalHandlePickerAction> goal_handle);
    void initializeComponents();
};

} // namespace jetcobot_picker
