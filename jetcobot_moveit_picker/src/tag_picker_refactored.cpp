#include "jetcobot_moveit_picker/tag_picker_refactored.hpp"

namespace jetcobot_picker {

TagPicker::TagPicker() : Node("tag_picker") {
    RCLCPP_INFO(get_logger(), "Initializing TagPicker...");
    initializeComponents();
    
    // Create action server
    action_server_ = rclcpp_action::create_server<PickerAction>(
        this,
        "picker_action",
        std::bind(&TagPicker::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TagPicker::handle_cancel, this, std::placeholders::_1),
        std::bind(&TagPicker::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "TagPicker initialized successfully");
}

void TagPicker::initializeComponents() {
    // Initialize all component managers
    gripper_controller_ = std::make_shared<GripperController>(shared_from_this());
    tag_detection_manager_ = std::make_shared<TagDetectionManager>(shared_from_this());
    motion_planner_ = std::make_shared<MotionPlanner>(shared_from_this());
    pick_place_executor_ = std::make_shared<PickPlaceExecutor>(
        shared_from_this(), gripper_controller_, tag_detection_manager_, motion_planner_);
}

bool TagPicker::execute() {
    // Initialize motion planner after the object is fully constructed
    if (!motion_planner_->initialize()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize motion planner");
        return false;
    }
    
    gripper_controller_->openGripperToHoldingPosition();

    RCLCPP_INFO(get_logger(), "TagPicker ready to receive action commands!");
    
    // Keep the node alive to process callbacks
    rclcpp::spin(shared_from_this());
    
    return true;
}

rclcpp_action::GoalResponse TagPicker::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickerAction::Goal> goal) {
    
    RCLCPP_INFO(get_logger(), "Received goal request: command=%s, source_id=%d, target_id=%d", 
               goal->command.c_str(), goal->source_tag_id, goal->target_tag_id);
    
    // Accept all goals for now
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TagPicker::handle_cancel(
    const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TagPicker::handle_accepted(const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    // Execute goal in a separate thread
    std::thread{std::bind(&TagPicker::execute_goal, this, goal_handle)}.detach();
}

void TagPicker::execute_goal(const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing goal");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PickerAction::Feedback>();
    auto result = std::make_shared<PickerAction::Result>();
    
    bool success = false;
    
    if (goal->command == "HOME") {
        success = handleHomeCommand(goal_handle);
    }
    else if (goal->command == "SCAN") {
        success = handleScanCommand(goal_handle);
    }
    else if (goal->command == "SCAN_FRONT") {
        success = handleScanFrontCommand(goal_handle);
    }
    else if (goal->command == "SCAN_LEFT") {
        success = handleScanLeftCommand(goal_handle);
    }
    else if (goal->command == "SCAN_RIGHT") {
        success = handleScanRightCommand(goal_handle);
    }
    else if (goal->command == "PICK_AND_PLACE") {
        success = handlePickAndPlaceCommand(goal_handle);
    }
    else {
        RCLCPP_ERROR(get_logger(), "Unknown command: %s", goal->command.c_str());
        result->success = false;
        result->error_message = "Unknown command: " + goal->command;
        goal_handle->abort(result);
        return;
    }
    
    // Set final result
    result->success = success;
    if (!success) {
        result->error_message = "Command execution failed";
    }
    
    // Send final feedback
    feedback->current_phase = "completed";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    if (success) {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
    } else {
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "Goal aborted");
    }
}

bool TagPicker::handleHomeCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing HOME command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_home";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    bool success = motion_planner_->moveToConfiguration("ready_to_see");
    
    if (!success) {
        RCLCPP_ERROR(get_logger(), "Failed to move to home position");
    } else {
        RCLCPP_INFO(get_logger(), "HOME command completed successfully");
    }
    
    return success;
}

bool TagPicker::handleScanCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing SCAN command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "searching";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    bool success = tag_detection_manager_->collectDetectedTagsAndAcquireTransforms(TimingConstants::TAG_COLLECTION_TIME);
    
    if (!success) {
        RCLCPP_ERROR(get_logger(), "SCAN command failed - no tags detected");
    } else {
        tag_detection_manager_->printDetectedTags();
        RCLCPP_INFO(get_logger(), "SCAN command completed successfully");
    }
    
    return success;
}

bool TagPicker::executeScanWithConfiguration(const std::string& config_name, const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_scan_position";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    // Move to specified configuration
    if (!motion_planner_->moveToConfiguration(config_name)) {
        RCLCPP_ERROR(get_logger(), "Failed to move to %s configuration", config_name.c_str());
        return false;
    }
    
    feedback->current_phase = "searching";
    goal_handle->publish_feedback(feedback);
    
    bool success = tag_detection_manager_->collectDetectedTagsAndAcquireTransforms(TimingConstants::TAG_COLLECTION_TIME);
    
    if (!success) {
        RCLCPP_ERROR(get_logger(), "%s command failed - no tags detected", config_name.c_str());
    } else {
        tag_detection_manager_->printDetectedTags();
        RCLCPP_INFO(get_logger(), "%s command completed successfully", config_name.c_str());
    }
    
    return success;
}

bool TagPicker::handleScanFrontCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing SCAN_FRONT command");
    return executeScanWithConfiguration("scan_front", goal_handle);
}

bool TagPicker::handleScanLeftCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing SCAN_LEFT command");
    return executeScanWithConfiguration("scan_left", goal_handle);
}

bool TagPicker::handleScanRightCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing SCAN_RIGHT command");
    return executeScanWithConfiguration("scan_right", goal_handle);
}

bool TagPicker::handlePickAndPlaceCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle) {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "Executing PICK_AND_PLACE command: source_id=%d, target_id=%d, target_tf_name='%s'", 
               goal->source_tag_id, goal->target_tag_id, goal->target_tf_name.c_str());
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    
    // Execute pick operation
    feedback->current_phase = "approaching_source";
    feedback->current_tag_id = goal->source_tag_id;
    goal_handle->publish_feedback(feedback);
    
    feedback->current_phase = "picking";
    goal_handle->publish_feedback(feedback);
    
    if (!pick_place_executor_->executePick(goal->source_tag_id)) {
        RCLCPP_ERROR(get_logger(), "Pick operation failed for tag ID %d", goal->source_tag_id);
        return false;
    }
    
    // Execute place operation - check if using TF name or tag ID
    feedback->current_phase = "moving_to_target";
    if (!goal->target_tf_name.empty()) {
        // Use TF frame name
        feedback->current_tag_id = -1;  // No tag ID when using TF name
        goal_handle->publish_feedback(feedback);
        
        feedback->current_phase = "placing";
        goal_handle->publish_feedback(feedback);
        
        if (!pick_place_executor_->executePlace(goal->target_tf_name, goal->source_tag_id)) {
            RCLCPP_ERROR(get_logger(), "Place operation failed for TF frame %s", goal->target_tf_name.c_str());
            return false;
        }
    } else {
        // Use tag ID
        feedback->current_tag_id = goal->target_tag_id;
        goal_handle->publish_feedback(feedback);
        
        feedback->current_phase = "placing";
        goal_handle->publish_feedback(feedback);
        
        if (!pick_place_executor_->executePlace(goal->target_tag_id, goal->source_tag_id)) {
            RCLCPP_ERROR(get_logger(), "Place operation failed for tag ID %d", goal->target_tag_id);
            return false;
        }
    }
    
    RCLCPP_INFO(get_logger(), "PICK_AND_PLACE command completed successfully");
    return true;
}

} // namespace jetcobot_picker

// Main function
int main(int argc, char* argv[]) {
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create and execute TagPicker
    auto tag_picker = std::make_shared<jetcobot_picker::TagPicker>();
    
    bool success = tag_picker->execute();
    
    // Shutdown ROS
    rclcpp::shutdown();
    return success ? 0 : 1;
}
