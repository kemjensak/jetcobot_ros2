#include "jetcobot_moveit_picker/operations/pick_place_controller.hpp"
#include "jetcobot_moveit_picker/core/robot_controller.hpp"
#include "jetcobot_moveit_picker/core/tag_manager.hpp"
#include "jetcobot_moveit_picker/core/collision_manager.hpp"
#include <thread>

namespace jetcobot_picker {

PickPlaceController::PickPlaceController(
    rclcpp::Node* node,
    std::shared_ptr<RobotController> robot_controller,
    std::shared_ptr<TagManager> tag_manager,
    std::shared_ptr<CollisionManager> collision_manager)
    : node_(node), robot_controller_(robot_controller), tag_manager_(tag_manager), collision_manager_(collision_manager)
{
}

// ============================================================================
// MAIN PICK AND PLACE OPERATIONS
// ============================================================================

bool PickPlaceController::executePick(int tag_id)
{
    RCLCPP_INFO(node_->get_logger(), "Starting pick operation for tag ID: %d", tag_id);
    
    // Get the stored tag transform
    geometry_msgs::msg::TransformStamped tag_transform;
    if (!tag_manager_->getStoredTagTransform(tag_id, tag_transform)) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot find stored transform for tag ID %d", tag_id);
        return false;
    }

    // Move to tag position first
    if (!robot_controller_->moveToReacquireTagPosition(tag_transform, tag_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to tag position for tag %d", tag_id);
        return false;
    }
    
    // Get the potentially updated tag transform after approach
    geometry_msgs::msg::TransformStamped updated_tag_transform;
    if (tag_manager_->getStoredTagTransform(tag_id, updated_tag_transform)) {
        tag_transform = updated_tag_transform;
        RCLCPP_INFO(node_->get_logger(), "Using updated tag transform for final approach");
    }

    // Get current EEF orientation
    geometry_msgs::msg::Pose current_ee_pose = robot_controller_->getCurrentPose("TCP");
    auto current_ee_orientation = current_ee_pose.orientation;
    
    // Calculate multiple final target poses with different X-axis rotations
    auto final_target_poses = robot_controller_->calculateBaseAlignedPoses(tag_transform, RobotController::MovementConstants::PICK_HEIGHT);
    
    // Try each pose for final approach until one succeeds
    bool final_approach_success = false;
    geometry_msgs::msg::Pose successful_pose;
    const auto& angles = RobotController::RotationAngles::APPROACH_ANGLES;
    
    for (size_t i = 0; i < final_target_poses.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "Attempting final approach %zu/%zu (X-rotation: %d°)", 
                   i + 1, final_target_poses.size(), angles[i]);
        final_target_poses[i].orientation = current_ee_orientation;  // Maintain current EEF orientation
        
        // Move to final position using Cartesian path
        std::vector<geometry_msgs::msg::Pose> approach_waypoints{final_target_poses[i]};
        if (robot_controller_->executeCartesianPath(approach_waypoints, "final approach to tag")) {
            final_approach_success = true;
            successful_pose = final_target_poses[i];  // Store the successful pose
            RCLCPP_INFO(node_->get_logger(), "Final approach successful with %d° X-rotation!", angles[i]);
            break;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Final approach failed with %d° X-rotation, trying next angle...", angles[i]);
        }
    }
    
    if (!final_approach_success) {
        RCLCPP_ERROR(node_->get_logger(), "All final approach attempts failed!");
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Final aligned motion completed!");
    std::this_thread::sleep_for(std::chrono::milliseconds(RobotController::TimingConstants::STABILIZE_DELAY_MS));
    
    // Close gripper
    robot_controller_->closeGripperToPicking();

    // Immediately attach the picked object to gripper (removes world collision object and attaches to robot)
    RCLCPP_INFO(node_->get_logger(), "Attaching picked object to gripper...");
    collision_manager_->attachBoxToGripper(tag_id);
    
    // Remove from stored transforms to prevent re-creation of world collision object
    tag_manager_->removeStoredTagTransform(tag_id);

    // Lift object with the attached collision object
    if (!robot_controller_->executeLiftMovement(RobotController::MovementConstants::LIFT_HEIGHT)) {
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Pick operation completed for tag ID: %d", tag_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(RobotController::TimingConstants::OPERATION_DELAY_MS));
    
    return true;
}

bool PickPlaceController::executePlace(int target_tag_id, int source_tag_id)
{
    RCLCPP_INFO(node_->get_logger(), "Starting place operation: placing source tag %d at target tag %d position", source_tag_id, target_tag_id);
    
    // Get the stored target tag transform
    geometry_msgs::msg::TransformStamped target_tag_transform;
    if (!tag_manager_->getStoredTagTransform(target_tag_id, target_tag_transform)) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot find stored transform for target tag ID %d", target_tag_id);
        return false;
    }

    // Move to target tag position first with -10° approach angle
    if (!robot_controller_->moveToReacquireTagPosition(target_tag_transform, target_tag_id, -1)) {  // Index 5 = -10°
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to target tag position for tag %d", target_tag_id);
        return false;
    }
    
    // Get the potentially updated target tag transform after approach
    geometry_msgs::msg::TransformStamped updated_target_tag_transform;
    if (tag_manager_->getStoredTagTransform(target_tag_id, updated_target_tag_transform)) {
        target_tag_transform = updated_target_tag_transform;
        RCLCPP_INFO(node_->get_logger(), "Using updated target tag transform for placement");
    }
    
    // Get current EEF orientation
    geometry_msgs::msg::Pose current_ee_pose = robot_controller_->getCurrentPose("TCP");
    auto current_ee_orientation = current_ee_pose.orientation;

    // Calculate placement pose using (potentially updated) stored transform
    auto place_pose = robot_controller_->calculateBaseAlignedPose(target_tag_transform, RobotController::MovementConstants::PLACE_HEIGHT);
    place_pose.orientation = current_ee_orientation;  // Maintain current EEF orientation
    
    // Move down to placement position
    std::vector<geometry_msgs::msg::Pose> place_waypoints{place_pose};
    if (!robot_controller_->executeStabilizedMovement(place_waypoints, "moving to placement position")) {
        return false;
    }
    
    // Open gripper to release object
    robot_controller_->openGripperToHoldingPosition();

    // Detach the object from gripper and place it as collision object at new location
    RCLCPP_INFO(node_->get_logger(), "Detaching object from gripper and placing at new location...");
    collision_manager_->detachBoxFromGripper(source_tag_id);

    // Move up to lift position
    if (!robot_controller_->executeLiftMovement(RobotController::MovementConstants::APPROACH_HEIGHT)) {
        return false;
    }
    
    // Update stored tag pose after successful placement
    RCLCPP_INFO(node_->get_logger(), "Updating stored pose for placed source tag ID %d (now at target position)...", source_tag_id);
    tag_manager_->updateStoredTagIfVisible(source_tag_id);
    
    RCLCPP_INFO(node_->get_logger(), "Place operation completed: source tag %d placed at target tag %d position", source_tag_id, target_tag_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(RobotController::TimingConstants::OPERATION_DELAY_MS));
    
    return true;
}

bool PickPlaceController::executePlace(const std::string& target_tf_name, int source_tag_id)
{
    RCLCPP_INFO(node_->get_logger(), "Starting place operation: placing source tag %d at TF frame: %s", source_tag_id, target_tf_name.c_str());
    
    // Get transform for the specified TF frame
    geometry_msgs::msg::TransformStamped target_transform;
    
    // First, try to get stored pinky transform
    if (tag_manager_->getStoredPinkyTransform(target_tf_name, target_transform)) {
        RCLCPP_INFO(node_->get_logger(), "Using stored pinky transform for TF frame: %s at position: x=%.3f, y=%.3f, z=%.3f", 
                   target_tf_name.c_str(),
                   target_transform.transform.translation.x,
                   target_transform.transform.translation.y,
                   target_transform.transform.translation.z);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to find TF frame %s in stored pinky transforms", target_tf_name.c_str());
        return false;
    }

    // First, move to approach position (APPROACH_HEIGHT above the target frame)
    auto approach_pose = robot_controller_->calculateBaseAlignedPose(target_transform, RobotController::MovementConstants::APPROACH_HEIGHT);

    RCLCPP_INFO(node_->get_logger(), "Moving to approach position (%.1fcm above) for TF frame: x=%.3f, y=%.3f, z=%.3f",
               RobotController::MovementConstants::APPROACH_HEIGHT * 100, approach_pose.position.x,
               approach_pose.position.y, approach_pose.position.z);

    // Use the basic movement approach - this needs access to move_group_ interface
    // For now, let's use cartesian path approach
    std::vector<geometry_msgs::msg::Pose> approach_waypoints{approach_pose};
    if (!robot_controller_->executeStabilizedMovement(approach_waypoints, "moving to TF frame approach position")) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to approach position for TF frame");
        return false;
    }

    // Then, move down to final placement position using Cartesian path
    auto place_pose = robot_controller_->calculateBaseAlignedPose(target_transform, RobotController::MovementConstants::PLACE_HEIGHT);
    
    RCLCPP_INFO(node_->get_logger(), "Moving to final placement position using Cartesian path: x=%.3f, y=%.3f, z=%.3f",
               place_pose.position.x, place_pose.position.y, place_pose.position.z);
    
    // Move to placement position using Cartesian path
    std::vector<geometry_msgs::msg::Pose> place_waypoints{place_pose};
    if (!robot_controller_->executeStabilizedMovement(place_waypoints, "moving to TF frame placement position")) {
        return false;
    }
    
    // Open gripper to release object
    robot_controller_->openGripperToHoldingPosition();
    
    // Detach the object from gripper and place it as collision object at new location
    RCLCPP_INFO(node_->get_logger(), "Detaching object from gripper and placing at new location...");
    collision_manager_->detachBoxFromGripper(source_tag_id);
    
    // Move up to lift position after placing
    if (!robot_controller_->executeLiftMovement(RobotController::MovementConstants::APPROACH_HEIGHT)) {
        return false;
    }

    // Update stored tag pose after successful placement
    RCLCPP_INFO(node_->get_logger(), "Updating stored pose for placed source tag ID %d (now at TF frame position)...", source_tag_id);
    tag_manager_->updateStoredTagIfVisible(source_tag_id);

    RCLCPP_INFO(node_->get_logger(), "Place operation completed: source tag %d placed at TF frame: %s", source_tag_id, target_tf_name.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(RobotController::TimingConstants::OPERATION_DELAY_MS));
    
    return true;
}

} // namespace jetcobot_picker
