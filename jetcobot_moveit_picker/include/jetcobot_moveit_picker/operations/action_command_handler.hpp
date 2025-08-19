#ifndef ACTION_COMMAND_HANDLER_HPP
#define ACTION_COMMAND_HANDLER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <jetcobot_interfaces/action/picker_action.hpp>

// Forward declarations
namespace jetcobot_picker {
    class RobotController;
    class TagManager;
    class CollisionManager;
    class TransformManager;
    class PickPlaceController;
}

namespace jetcobot_picker {

/**
 * @brief Handles ROS action server commands for the picker system
 */
class ActionCommandHandler 
{
public:
    using PickerAction = jetcobot_interfaces::action::PickerAction;
    using GoalHandlePickerAction = rclcpp_action::ServerGoalHandle<PickerAction>;

    /**
     * @brief Constructor
     * @param node ROS node pointer for logging
     * @param robot_controller Robot movement controller
     * @param tag_manager Tag detection and management
     * @param collision_manager Collision object management
     * @param transform_manager Transform frame management
     * @param pick_place_controller Pick and place operations
     */
    ActionCommandHandler(
        rclcpp::Node* node,
        std::shared_ptr<RobotController> robot_controller,
        std::shared_ptr<TagManager> tag_manager,
        std::shared_ptr<CollisionManager> collision_manager,
        std::shared_ptr<TransformManager> transform_manager,
        std::shared_ptr<PickPlaceController> pick_place_controller
    );

    // ============================================================================
    // COMMAND HANDLERS
    // ============================================================================

    /**
     * @brief Handle HOME command - move robot to ready position
     */
    bool handleHomeCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN command - scan for tags from current position
     */
    bool handleScanCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN_FRONT command - move to front scan position and scan
     */
    bool handleScanFrontCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN_LEFT command - move to left scan position and scan
     */
    bool handleScanLeftCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN_RIGHT command - move to right scan position and scan
     */
    bool handleScanRightCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN_PINKY command - approach specific tag and store tag + pinky loadpoint poses
     * @param goal_handle Action goal handle
     */
    bool handleScanPinkyCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle CLEAR_PINKY command - remove all pinky-related static transforms and collision objects
     * @param goal_handle Action goal handle
     */
    bool handleClearPinkyCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle PICK_AND_PLACE command - execute complete pick and place operation
     * @param goal_handle Action goal handle
     */
    bool handlePickAndPlaceCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

private:
    rclcpp::Node* node_;
    std::shared_ptr<RobotController> robot_controller_;
    std::shared_ptr<TagManager> tag_manager_;
    std::shared_ptr<CollisionManager> collision_manager_;
    std::shared_ptr<TransformManager> transform_manager_;
    std::shared_ptr<PickPlaceController> pick_place_controller_;
};

} // namespace jetcobot_picker

#endif // ACTION_COMMAND_HANDLER_HPP
