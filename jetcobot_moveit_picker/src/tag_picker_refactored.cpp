#include "jetcobot_moveit_picker/tag_picker_refactored.hpp"
#include <thread>

namespace jetcobot_picker {

TagPicker::TagPicker() : Node("tag_picker"), 
                        tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
                        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
    // Create gripper command publisher
    gripper_pub_ = create_publisher<std_msgs::msg::Int32>("/gripper_command", 10);
    
    // Create action server
    action_server_ = rclcpp_action::create_server<PickerAction>(
        this,
        "picker_action",
        std::bind(&TagPicker::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TagPicker::handle_cancel, this, std::placeholders::_1),
        std::bind(&TagPicker::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "TagPicker initialized successfully");
}

bool TagPicker::execute()
{
    // Setup MoveIt interfaces
    setupMoveItInterfaces();
    
    // Initialize all modular components
    initializeComponents();

    // Initialize gripper position
    robot_controller_->openGripperToHoldingPosition();

    RCLCPP_INFO(get_logger(), "TagPicker ready to receive action commands!");
    
    // Keep the node alive to process callbacks
    rclcpp::spin(shared_from_this());
    
    return true;
}

// ============================================================================
// ACTION SERVER CALLBACKS
// ============================================================================

rclcpp_action::GoalResponse TagPicker::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickerAction::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request: command=%s, source_id=%d, target_id=%d", 
               goal->command.c_str(), goal->source_tag_id, goal->target_tag_id);
    
    // Accept all goals for now
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TagPicker::handle_cancel(
    const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TagPicker::handle_accepted(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    // Execute goal in a separate thread
    std::thread{std::bind(&TagPicker::execute_goal, this, goal_handle)}.detach();
}

void TagPicker::execute_goal(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing goal");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PickerAction::Feedback>();
    auto result = std::make_shared<PickerAction::Result>();
    
    bool success = false;
    
    if (goal->command == "HOME") {
        success = command_handler_->handleHomeCommand(goal_handle);
    }
    else if (goal->command == "SCAN") {
        success = command_handler_->handleScanCommand(goal_handle);
    }
    else if (goal->command == "SCAN_FRONT") {
        success = command_handler_->handleScanFrontCommand(goal_handle);
    }
    else if (goal->command == "SCAN_LEFT") {
        success = command_handler_->handleScanLeftCommand(goal_handle);
    }
    else if (goal->command == "SCAN_RIGHT") {
        success = command_handler_->handleScanRightCommand(goal_handle);
    }
    else if (goal->command == "SCAN_PINKY") {
        success = command_handler_->handleScanPinkyCommand(goal_handle);
    }
    else if (goal->command == "CLEAR_PINKY") {
        success = command_handler_->handleClearPinkyCommand(goal_handle);
    }
    else if (goal->command == "PICK_AND_PLACE") {
        success = command_handler_->handlePickAndPlaceCommand(goal_handle);
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

// ============================================================================
// INITIALIZATION HELPERS
// ============================================================================

void TagPicker::setupMoveItInterfaces()
{
    // Initialize MoveGroupInterface after the object is fully constructed
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    move_group_interface_->setMaxAccelerationScalingFactor(0.9);
    move_group_interface_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_->setPlanningTime(10.0);  // Set planning time to 10 seconds
    move_group_interface_->setNumPlanningAttempts(200);
    
    // Create planning scene interface
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    RCLCPP_INFO(get_logger(), "MoveIt interfaces initialized");
}

void TagPicker::initializeComponents()
{
    // Create core components
    robot_controller_ = std::make_shared<RobotController>(this, move_group_interface_, gripper_pub_);
    tag_manager_ = std::make_shared<TagManager>(this, tf_buffer_);
    collision_manager_ = std::make_shared<CollisionManager>(this, planning_scene_interface_, tf_buffer_);
    transform_manager_ = std::make_shared<TransformManager>(this, tf_buffer_);
    
    // Create operation controllers
    pick_place_controller_ = std::make_shared<PickPlaceController>(
        this, robot_controller_, tag_manager_, collision_manager_);
    
    command_handler_ = std::make_shared<ActionCommandHandler>(
        this, robot_controller_, tag_manager_, collision_manager_, 
        transform_manager_, pick_place_controller_);
    
    // Initialize components that need setup
    tag_manager_->initialize();
    transform_manager_->initialize();
    
    RCLCPP_INFO(get_logger(), "All modular components initialized");
}

} // namespace jetcobot_picker

// Main function
int main(int argc, char* argv[])
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create and execute TagPicker
    auto tag_picker = std::make_shared<jetcobot_picker::TagPicker>();
    
    bool success = tag_picker->execute();
    
    // Shutdown ROS
    rclcpp::shutdown();
    return success ? 0 : 1;
}
