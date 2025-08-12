#include "jetcobot_moveit_picker/pick_place_executor.hpp"
#include <thread>

namespace jetcobot_picker {

PickPlaceExecutor::PickPlaceExecutor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<GripperController> gripper_controller,
    std::shared_ptr<TagDetectionManager> tag_detection_manager,
    std::shared_ptr<MotionPlanner> motion_planner
) : node_(node), 
    gripper_controller_(gripper_controller),
    tag_detection_manager_(tag_detection_manager),
    motion_planner_(motion_planner) {
    
    RCLCPP_INFO(node_->get_logger(), "PickPlaceExecutor initialized");
}

bool PickPlaceExecutor::executePick(int tag_id) {
    RCLCPP_INFO(node_->get_logger(), "Starting pick operation for tag ID: %d", tag_id);
    
    // Get the stored tag transform
    geometry_msgs::msg::TransformStamped tag_transform;
    if (!tag_detection_manager_->getStoredTagTransform(tag_id, tag_transform)) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot find stored transform for tag ID %d", tag_id);
        return false;
    }

    // Move to tag position first
    if (!motion_planner_->moveToReacquireTagPosition(tag_transform, tag_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to tag position for tag %d", tag_id);
        return false;
    }
    
    // Get the potentially updated tag transform after approach
    geometry_msgs::msg::TransformStamped updated_tag_transform;
    if (tag_detection_manager_->getStoredTagTransform(tag_id, updated_tag_transform)) {
        tag_transform = updated_tag_transform;
        RCLCPP_INFO(node_->get_logger(), "Using updated tag transform for final approach");
    }

    return performPickSequence(tag_transform);
}

bool PickPlaceExecutor::performPickSequence(const geometry_msgs::msg::TransformStamped& tag_transform) {
    // Get current EEF orientation
    geometry_msgs::msg::Pose current_ee_pose = motion_planner_->getCurrentPose("TCP");
    auto current_ee_orientation = current_ee_pose.orientation;
    
    // Calculate multiple final target poses with different X-axis rotations
    auto final_target_poses = motion_planner_->calculateBaseAlignedPoses(tag_transform, MovementConstants::PICK_HEIGHT);
    
    // Try each pose for final approach until one succeeds
    bool final_approach_success = false;
    const auto& angles = RotationAngles::APPROACH_ANGLES;
    
    for (size_t i = 0; i < final_target_poses.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "Attempting final approach %zu/%zu (X-rotation: %d°)", 
                   i + 1, final_target_poses.size(), angles[i]);
        final_target_poses[i].orientation = current_ee_orientation;  // Maintain current EEF orientation
        
        // Move to final position using Cartesian path
        std::vector<geometry_msgs::msg::Pose> approach_waypoints{final_target_poses[i]};
        if (motion_planner_->executeCartesianPath(approach_waypoints, "final approach to tag")) {
            final_approach_success = true;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::STABILIZE_DELAY_MS));
    
    // Close gripper
    gripper_controller_->closeGripperToPicking();

    // Lift object
    if (!motion_planner_->executeLiftMovement(MovementConstants::LIFT_HEIGHT)) {
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Pick operation completed successfully");
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    
    return true;
}

bool PickPlaceExecutor::executePlace(int target_tag_id, int source_tag_id) {
    RCLCPP_INFO(node_->get_logger(), "Starting place operation: placing source tag %d at target tag %d position", source_tag_id, target_tag_id);
    
    // Get the stored target tag transform
    geometry_msgs::msg::TransformStamped target_tag_transform;
    if (!tag_detection_manager_->getStoredTagTransform(target_tag_id, target_tag_transform)) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot find stored transform for target tag ID %d", target_tag_id);
        return false;
    }

    // Move to target tag position first
    if (!motion_planner_->moveToReacquireTagPosition(target_tag_transform, target_tag_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to target tag position for tag %d", target_tag_id);
        return false;
    }
    
    // Get the potentially updated target tag transform after approach
    geometry_msgs::msg::TransformStamped updated_target_tag_transform;
    if (tag_detection_manager_->getStoredTagTransform(target_tag_id, updated_target_tag_transform)) {
        target_tag_transform = updated_target_tag_transform;
        RCLCPP_INFO(node_->get_logger(), "Using updated target tag transform for placement");
    }
    
    return performPlaceSequence(target_tag_transform, source_tag_id);
}

bool PickPlaceExecutor::executePlace(const std::string& target_tf_name, int source_tag_id) {
    RCLCPP_INFO(node_->get_logger(), "Starting place operation: placing source tag %d at TF frame: %s", source_tag_id, target_tf_name.c_str());
    
    // Get transform for the specified TF frame
    geometry_msgs::msg::TransformStamped target_transform;
    try {
        // This would need access to tf_buffer, we'll get it from tag_detection_manager for now
        // In a real implementation, you might want to pass tf_buffer separately or create a TF manager class
        if (!tag_detection_manager_->findSpecificAprilTag(source_tag_id, target_transform)) {
            // Try to get TF frame directly (this is a simplified approach)
            RCLCPP_ERROR(node_->get_logger(), "Failed to find TF frame %s", target_tf_name.c_str());
            return false;
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to find TF frame %s: %s", target_tf_name.c_str(), e.what());
        return false;
    }

    // First, move to approach position
    auto approach_pose = motion_planner_->calculateBaseAlignedPose(target_transform, MovementConstants::APPROACH_HEIGHT);

    RCLCPP_INFO(node_->get_logger(), "Moving to approach position for TF frame: x=%.3f, y=%.3f, z=%.3f",
               approach_pose.position.x, approach_pose.position.y, approach_pose.position.z);

    // This would need access to move_group_interface, we'll use motion_planner methods
    std::vector<geometry_msgs::msg::Pose> approach_waypoints{approach_pose};
    if (!motion_planner_->executeStabilizedMovement(approach_waypoints, "approach to TF frame")) {
        return false;
    }

    return performPlaceSequence(target_transform, source_tag_id);
}

bool PickPlaceExecutor::performPlaceSequence(const geometry_msgs::msg::TransformStamped& target_transform, int source_tag_id) {
    // Get current EEF orientation
    geometry_msgs::msg::Pose current_ee_pose = motion_planner_->getCurrentPose("TCP");
    auto current_ee_orientation = current_ee_pose.orientation;

    // Calculate placement pose using stored transform
    auto place_pose = motion_planner_->calculateBaseAlignedPose(target_transform, MovementConstants::PLACE_HEIGHT);
    place_pose.orientation = current_ee_orientation;  // Maintain current EEF orientation
    
    // Move down to placement position
    std::vector<geometry_msgs::msg::Pose> place_waypoints{place_pose};
    if (!motion_planner_->executeStabilizedMovement(place_waypoints, "moving to placement position")) {
        return false;
    }
    
    // Open gripper to release object
    gripper_controller_->openGripperToHoldingPosition();

    // Move up to lift position
    if (!motion_planner_->executeLiftMovement(MovementConstants::APPROACH_HEIGHT)) {
        return false;
    }
    
    // Update stored tag pose after successful placement
    RCLCPP_INFO(node_->get_logger(), "Updating stored pose for placed source tag ID %d...", source_tag_id);
    tag_detection_manager_->updateStoredTagIfVisible(source_tag_id);
    
    RCLCPP_INFO(node_->get_logger(), "Place operation completed successfully");
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    
    return true;
}

} // namespace jetcobot_picker
