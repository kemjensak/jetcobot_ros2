#ifndef TAG_PICKER_REFACTORED_HPP
#define TAG_PICKER_REFACTORED_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/int32.hpp>
#include <jetcobot_interfaces/action/picker_action.hpp>

// Include modular components
#include "jetcobot_moveit_picker/core/robot_controller.hpp"
#include "jetcobot_moveit_picker/core/tag_manager.hpp"
#include "jetcobot_moveit_picker/core/collision_manager.hpp"
#include "jetcobot_moveit_picker/core/transform_manager.hpp"
#include "jetcobot_moveit_picker/operations/pick_place_controller.hpp"
#include "jetcobot_moveit_picker/operations/action_command_handler.hpp"

namespace jetcobot_picker {

/**
 * @brief Main TagPicker class that orchestrates all picker functionality
 * 
 * This class follows the Facade pattern, providing a unified interface to the
 * complex subsystem of robot control, tag management, collision handling, etc.
 */
class TagPicker : public rclcpp::Node
{
public:
    using PickerAction = jetcobot_interfaces::action::PickerAction;
    using GoalHandlePickerAction = rclcpp_action::ServerGoalHandle<PickerAction>;

    /**
     * @brief Constructor for TagPicker
     */
    TagPicker();

    /**
     * @brief Main execution function
     * @return true if execution successful, false otherwise
     */
    bool execute();

private:
    // ============================================================================
    // CORE COMPONENTS
    // ============================================================================
    
    // Core MoveIt and ROS interfaces
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publishers and action server
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;
    rclcpp_action::Server<PickerAction>::SharedPtr action_server_;
    
    // Modular components - using composition over inheritance
    std::shared_ptr<RobotController> robot_controller_;
    std::shared_ptr<TagManager> tag_manager_;
    std::shared_ptr<CollisionManager> collision_manager_;
    std::shared_ptr<TransformManager> transform_manager_;
    std::shared_ptr<PickPlaceController> pick_place_controller_;
    std::shared_ptr<ActionCommandHandler> command_handler_;

    // ============================================================================
    // ACTION SERVER CALLBACKS
    // ============================================================================
    
    /**
     * @brief Goal callback for action server
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickerAction::Goal> goal);

    /**
     * @brief Cancel callback for action server
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Accept callback for action server
     */
    void handle_accepted(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Execute the action goal
     */
    void execute_goal(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    // ============================================================================
    // INITIALIZATION HELPERS
    // ============================================================================
    
    /**
     * @brief Initialize all modular components
     */
    void initializeComponents();

    /**
     * @brief Setup MoveIt interfaces
     */
    void setupMoveItInterfaces();
};

} // namespace jetcobot_picker

#endif // TAG_PICKER_REFACTORED_HPP
