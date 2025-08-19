#include "jetcobot_moveit_picker/operations/action_command_handler.hpp"
#include "jetcobot_moveit_picker/core/robot_controller.hpp"
#include "jetcobot_moveit_picker/core/tag_manager.hpp"
#include "jetcobot_moveit_picker/core/collision_manager.hpp"
#include "jetcobot_moveit_picker/core/transform_manager.hpp"
#include "jetcobot_moveit_picker/operations/pick_place_controller.hpp"

namespace jetcobot_picker {

ActionCommandHandler::ActionCommandHandler(
    rclcpp::Node* node,
    std::shared_ptr<RobotController> robot_controller,
    std::shared_ptr<TagManager> tag_manager,
    std::shared_ptr<CollisionManager> collision_manager,
    std::shared_ptr<TransformManager> transform_manager,
    std::shared_ptr<PickPlaceController> pick_place_controller)
    : node_(node), robot_controller_(robot_controller), tag_manager_(tag_manager),
      collision_manager_(collision_manager), transform_manager_(transform_manager),
      pick_place_controller_(pick_place_controller)
{
}

// ============================================================================
// COMMAND HANDLERS
// ============================================================================

bool ActionCommandHandler::handleHomeCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing HOME command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_home";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    bool success = robot_controller_->moveToConfiguration("ready_to_see");
    
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to home position");
    } else {
        RCLCPP_INFO(node_->get_logger(), "HOME command completed successfully");
    }
    
    return success;
}

bool ActionCommandHandler::handleScanCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing SCAN command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "searching";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    bool success = tag_manager_->collectDetectedTagsAndAcquireTransforms(RobotController::TimingConstants::TAG_COLLECTION_TIME);
    
    // Always update collision objects regardless of success/failure to remove old boxes
    collision_manager_->publishBoxCollisionObjects(tag_manager_->getAllStoredTagTransforms());
    tag_manager_->printDetectedTags();
    
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "SCAN command failed - no tags detected");
    } else {
        RCLCPP_INFO(node_->get_logger(), "SCAN command completed successfully");
    }
    
    return success;
}

bool ActionCommandHandler::handleScanFrontCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing SCAN_FRONT command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_scan_position";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    // Move to scan_front configuration
    if (!robot_controller_->moveToConfiguration("scan_front")) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to scan_front configuration");
        return false;
    }
    
    feedback->current_phase = "searching";
    goal_handle->publish_feedback(feedback);
    
    bool success = tag_manager_->collectDetectedTagsAndAcquireTransforms(RobotController::TimingConstants::TAG_COLLECTION_TIME);
    
    // Always update collision objects regardless of success/failure to remove old boxes
    collision_manager_->publishBoxCollisionObjects(tag_manager_->getAllStoredTagTransforms());
    tag_manager_->printDetectedTags();
    
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "SCAN_FRONT command failed - no tags detected");
    } else {
        RCLCPP_INFO(node_->get_logger(), "SCAN_FRONT command completed successfully");
    }
    
    return success;
}

bool ActionCommandHandler::handleScanLeftCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing SCAN_LEFT command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_scan_position";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    // Move to scan_left configuration
    if (!robot_controller_->moveToConfiguration("scan_left")) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to scan_left configuration");
        return false;
    }
    
    feedback->current_phase = "searching";
    goal_handle->publish_feedback(feedback);
    
    bool success = tag_manager_->collectDetectedTagsAndAcquireTransforms(RobotController::TimingConstants::TAG_COLLECTION_TIME);
    
    // Always update collision objects regardless of success/failure to remove old boxes
    collision_manager_->publishBoxCollisionObjects(tag_manager_->getAllStoredTagTransforms());
    tag_manager_->printDetectedTags();
    
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "SCAN_LEFT command failed - no tags detected");
    } else {
        RCLCPP_INFO(node_->get_logger(), "SCAN_LEFT command completed successfully");
    }
    
    return success;
}

bool ActionCommandHandler::handleScanRightCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing SCAN_RIGHT command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_scan_position";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    // Move to scan_right configuration
    if (!robot_controller_->moveToConfiguration("scan_right")) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to scan_right configuration");
        return false;
    }
    
    feedback->current_phase = "searching";
    goal_handle->publish_feedback(feedback);
    
    bool success = tag_manager_->collectDetectedTagsAndAcquireTransforms(RobotController::TimingConstants::TAG_COLLECTION_TIME);
    
    // Always update collision objects regardless of success/failure to remove old boxes
    collision_manager_->publishBoxCollisionObjects(tag_manager_->getAllStoredTagTransforms());
    tag_manager_->printDetectedTags();
    
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "SCAN_RIGHT command failed - no tags detected");
    } else {
        RCLCPP_INFO(node_->get_logger(), "SCAN_RIGHT command completed successfully");
    }
    
    return success;
}

bool ActionCommandHandler::handleScanPinkyCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing SCAN_PINKY command for stored tags 31, 32, 33");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    
    // Define target tag IDs for pinky scanning
    std::vector<int> target_tags = {31, 32, 33};
    std::vector<int> processed_tags;
    bool overall_success = false;
    
    for (int tag_id : target_tags) {
        // Check if we have a stored transform for this tag
        geometry_msgs::msg::TransformStamped tag_transform;
        if (!tag_manager_->getStoredTagTransform(tag_id, tag_transform)) {
            RCLCPP_INFO(node_->get_logger(), "No stored transform for tag ID %d, skipping...", tag_id);
            continue;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Processing pinky scan for tag ID: %d", tag_id);
        
        feedback->current_phase = "pinky_scanning";
        feedback->current_tag_id = tag_id;
        goal_handle->publish_feedback(feedback);
        
        // Move to the tag's position to get a more precise pose
        feedback->current_phase = "approaching_target";
        goal_handle->publish_feedback(feedback);
        
        if (!robot_controller_->moveToReacquireTagPosition(tag_transform, tag_id)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to move to scan position for tag %d", tag_id);
            continue; // Skip this tag and try next one
        }
        
        // Wait a moment for the tag detection to stabilize
        feedback->current_phase = "updating_poses";
        goal_handle->publish_feedback(feedback);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(RobotController::TimingConstants::STABILIZE_DELAY_MS));
        
        // Try to update the stored tag pose with more precise data
        bool tag_update_success = tag_manager_->updateStoredTagIfVisible(tag_id);
        
        if (tag_update_success) {
            // Get the updated transform to log the new position
            geometry_msgs::msg::TransformStamped updated_transform;
            if (tag_manager_->getStoredTagTransform(tag_id, updated_transform)) {
                RCLCPP_INFO(node_->get_logger(), "Updated tag %d position: x=%.3f, y=%.3f, z=%.3f", 
                           tag_id,
                           updated_transform.transform.translation.x,
                           updated_transform.transform.translation.y,
                           updated_transform.transform.translation.z);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "Could not update tag pose for tag ID %d", tag_id);
        }
        
        // Publish ground-projected transforms for pinky frames first
        if (tag_update_success) {
            transform_manager_->publishGroundProjectedTransforms(tag_id);
        }
        
        // Store pinky loadpoint transforms after ground-projected transforms
        bool pinky_store_success = tag_manager_->storePinkyLoadpointTransforms(tag_id);
        
        if (pinky_store_success) {
            RCLCPP_INFO(node_->get_logger(), "Successfully stored pinky loadpoint transforms for tag %d", tag_id);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Could not store all pinky loadpoint transforms for tag %d", tag_id);
        }
        
        // Consider success if either tag or pinky transforms were stored
        bool tag_success = tag_update_success || pinky_store_success;
        
        if (tag_success) {
            // Create collision objects at pinky bag poses (only if tag is visible)
            if (tag_update_success) {
                collision_manager_->createCollisionObjectsAtPinkyBagPoses(tag_id);
            }
            
            processed_tags.push_back(tag_id);
            overall_success = true;
            RCLCPP_INFO(node_->get_logger(), "SCAN_PINKY completed successfully for tag ID %d", tag_id);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "SCAN_PINKY failed for tag ID %d", tag_id);
        }
    }
    
    // Summary of results
    if (overall_success) {
        RCLCPP_INFO(node_->get_logger(), "SCAN_PINKY command completed. Processed %zu out of %zu target tags.", 
                   processed_tags.size(), target_tags.size());
        
        if (!processed_tags.empty()) {
            std::string processed_list = "Processed tags: ";
            for (size_t i = 0; i < processed_tags.size(); ++i) {
                processed_list += std::to_string(processed_tags[i]);
                if (i < processed_tags.size() - 1) processed_list += ", ";
            }
            RCLCPP_INFO(node_->get_logger(), "%s", processed_list.c_str());
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "SCAN_PINKY failed - no tags could be processed. Make sure tags 31, 32, or 33 are scanned first.");
        return false;
    }
    
    return overall_success;
}

bool ActionCommandHandler::handleClearPinkyCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing CLEAR_PINKY command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "clearing_pinky_data";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    // Clear all pinky-related static transforms
    transform_manager_->clearAllPinkyStaticTransforms();
    
    // Clear all pinky-related collision objects
    collision_manager_->clearAllPinkyCollisionObjects();
    
    // Clear stored pinky transforms
    tag_manager_->clearAllStoredPinkyTransforms();
    
    RCLCPP_INFO(node_->get_logger(), "CLEAR_PINKY command completed successfully");
    
    return true;
}

bool ActionCommandHandler::handlePickAndPlaceCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(node_->get_logger(), "Executing PICK_AND_PLACE command: source_id=%d, target_id=%d, target_tf_name='%s'", 
               goal->source_tag_id, goal->target_tag_id, goal->target_tf_name.c_str());
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    
    // Execute pick operation
    feedback->current_phase = "approaching_source";
    feedback->current_tag_id = goal->source_tag_id;
    goal_handle->publish_feedback(feedback);
    
    feedback->current_phase = "picking";
    goal_handle->publish_feedback(feedback);
    
    if (!pick_place_controller_->executePick(goal->source_tag_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Pick operation failed for tag ID %d", goal->source_tag_id);
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
        
        if (!pick_place_controller_->executePlace(goal->target_tf_name, goal->source_tag_id)) {
            RCLCPP_ERROR(node_->get_logger(), "Place operation failed for TF frame %s", goal->target_tf_name.c_str());
            return false;
        }
    } else {
        // Use tag ID
        feedback->current_tag_id = goal->target_tag_id;
        goal_handle->publish_feedback(feedback);
        
        feedback->current_phase = "placing";
        goal_handle->publish_feedback(feedback);
        
        if (!pick_place_controller_->executePlace(goal->target_tag_id, goal->source_tag_id)) {
            RCLCPP_ERROR(node_->get_logger(), "Place operation failed for tag ID %d", goal->target_tag_id);
            return false;
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "PICK_AND_PLACE command completed successfully");
    return true;
}

} // namespace jetcobot_picker
