#include "jetcobot_moveit_picker/core/tag_manager.hpp"
#include <thread>

namespace jetcobot_picker {

TagManager::TagManager(rclcpp::Node* node, std::unique_ptr<tf2_ros::Buffer>& tf_buffer)
    : node_(node), tf_buffer_(tf_buffer), is_collecting_detections_(false)
{
}

void TagManager::initialize()
{
    // Create subscriber for AprilTag detections
    detection_sub_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", 10,
        std::bind(&TagManager::detectionCallback, this, std::placeholders::_1));
}

// ============================================================================
// TAG DETECTION AND MANAGEMENT
// ============================================================================

bool TagManager::collectDetectedTagsAndAcquireTransforms(double collection_time_seconds)
{
    RCLCPP_INFO(node_->get_logger(), "Starting tag detection collection for %.1f seconds...", collection_time_seconds);
    
    // Clear previous detections
    detected_tag_ids_.clear();
    
    // Start collection
    is_collecting_detections_ = true;
    detection_start_time_ = std::chrono::steady_clock::now();
    
    // Wait for the specified collection time
    auto collection_duration = std::chrono::duration<double>(collection_time_seconds);
    std::this_thread::sleep_for(collection_duration);
    
    // Stop collection
    is_collecting_detections_ = false;
    
    RCLCPP_INFO(node_->get_logger(), "Tag detection collection completed. Found %zu unique tags.", detected_tag_ids_.size());
    
    if (detected_tag_ids_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No AprilTags detected during collection period!");
        return false;
    }
    
    // Acquire transforms for all detected tags
    int successful_transforms = 0;
    for (int tag_id : detected_tag_ids_) {
        std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
        
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link", tag_frame, tf2::TimePointZero, std::chrono::seconds(1));
            
            stored_tag_transforms_[tag_id] = transform;
            successful_transforms++;
            
            RCLCPP_INFO(node_->get_logger(), "Acquired transform for tag ID %d: x=%.3f, y=%.3f, z=%.3f", 
                       tag_id,
                       transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(node_->get_logger(), "Failed to get transform for tag ID %d: %s", tag_id, ex.what());
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Successfully acquired %d out of %zu tag transforms", 
               successful_transforms, detected_tag_ids_.size());
    
    return successful_transforms > 0;
}

std::set<int> TagManager::getDetectedTagIds() const
{
    return detected_tag_ids_;
}

void TagManager::printDetectedTags() const
{
    RCLCPP_INFO(node_->get_logger(), "Currently detected tag IDs (%zu total):", detected_tag_ids_.size());
    for (int tag_id : detected_tag_ids_) {
        auto it = stored_tag_transforms_.find(tag_id);
        if (it != stored_tag_transforms_.end()) {
            const auto& transform = it->second.transform;
            RCLCPP_INFO(node_->get_logger(), "  Tag %d: x=%.3f, y=%.3f, z=%.3f", 
                       tag_id, transform.translation.x, transform.translation.y, transform.translation.z);
        } else {
            RCLCPP_INFO(node_->get_logger(), "  Tag %d: (no stored transform)", tag_id);
        }
    }
}

bool TagManager::updateStoredTagIfVisible(int tag_id)
{
    RCLCPP_INFO(node_->get_logger(), "Checking if tag ID %d is visible for pose update...", tag_id);
    
    std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
    
    try {
        geometry_msgs::msg::TransformStamped updated_transform = tf_buffer_->lookupTransform(
            "base_link", tag_frame, tf2::TimePointZero, std::chrono::seconds(1));
        
        // Update the stored transform with the new, more accurate data
        stored_tag_transforms_[tag_id] = updated_transform;
        
        RCLCPP_INFO(node_->get_logger(), "Updated stored transform for tag ID %d: x=%.3f, y=%.3f, z=%.3f", 
                   tag_id,
                   updated_transform.transform.translation.x,
                   updated_transform.transform.translation.y,
                   updated_transform.transform.translation.z);
        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "Tag ID %d not visible from current position: %s", tag_id, ex.what());
        return false;
    }
}

bool TagManager::findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform)
{
    std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
    
    try {
        tag_transform = tf_buffer_->lookupTransform(
            "base_link", tag_frame, tf2::TimePointZero, std::chrono::seconds(1));
        
        RCLCPP_INFO(node_->get_logger(), "Found specific AprilTag: %s", tag_frame.c_str());
        RCLCPP_INFO(node_->get_logger(), "Tag position: x=%.3f, y=%.3f, z=%.3f", 
                   tag_transform.transform.translation.x,
                   tag_transform.transform.translation.y,
                   tag_transform.transform.translation.z);
        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to find AprilTag ID %d: %s", tag_id, ex.what());
        return false;
    }
}

// ============================================================================
// TRANSFORM STORAGE AND RETRIEVAL
// ============================================================================

bool TagManager::getStoredTagTransform(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform)
{
    auto it = stored_tag_transforms_.find(tag_id);
    if (it != stored_tag_transforms_.end()) {
        tag_transform = it->second;
        RCLCPP_INFO(node_->get_logger(), "Retrieved stored transform for tag ID %d", tag_id);
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "No stored transform found for tag ID %d", tag_id);
        return false;
    }
}

void TagManager::storeTagTransform(int tag_id, const geometry_msgs::msg::TransformStamped& transform)
{
    stored_tag_transforms_[tag_id] = transform;
    RCLCPP_INFO(node_->get_logger(), "Stored transform for tag ID %d", tag_id);
}

void TagManager::removeStoredTagTransform(int tag_id)
{
    auto it = stored_tag_transforms_.find(tag_id);
    if (it != stored_tag_transforms_.end()) {
        stored_tag_transforms_.erase(it);
        RCLCPP_INFO(node_->get_logger(), "Removed stored transform for tag ID %d", tag_id);
    }
}

void TagManager::clearAllStoredTagTransforms()
{
    stored_tag_transforms_.clear();
    RCLCPP_INFO(node_->get_logger(), "Cleared all stored tag transforms");
}

bool TagManager::getStoredPinkyTransform(const std::string& tf_name, geometry_msgs::msg::TransformStamped& pinky_transform)
{
    auto it = stored_pinky_transforms_.find(tf_name);
    if (it != stored_pinky_transforms_.end()) {
        pinky_transform = it->second;
        RCLCPP_INFO(node_->get_logger(), "Retrieved stored pinky transform for TF: %s", tf_name.c_str());
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "No stored pinky transform found for TF: %s", tf_name.c_str());
        return false;
    }
}

bool TagManager::storePinkyLoadpointTransforms(int tag_id)
{
    RCLCPP_INFO(node_->get_logger(), "Storing pinky loadpoint transforms for tag ID: %d", tag_id);
    
    // Determine pinky namespace based on tag ID
    std::string pinky_namespace;
    if (tag_id == 31) {
        pinky_namespace = "pinky1";
    } else if (tag_id == 32) {
        pinky_namespace = "pinky2";
    } else if (tag_id == 33) {
        pinky_namespace = "pinky3";
    } else {
        RCLCPP_WARN(node_->get_logger(), "Unknown tag ID %d, using default pinky1", tag_id);
        pinky_namespace = "pinky1";
    }
    
    RCLCPP_INFO(node_->get_logger(), "Using pinky namespace: %s for tag ID: %d", pinky_namespace.c_str(), tag_id);
    
    // List of pinky loadpoint TF frames to store
    std::vector<std::string> pinky_frames = {
        pinky_namespace + "/fl",
        pinky_namespace + "/fr", 
        pinky_namespace + "/rl",
        pinky_namespace + "/rr"
    };
    
    int successful_stores = 0;
    
    for (const std::string& frame_name : pinky_frames) {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link", frame_name, tf2::TimePointZero, std::chrono::seconds(1));
            
            stored_pinky_transforms_[frame_name] = transform;
            successful_stores++;
            
            RCLCPP_INFO(node_->get_logger(), "Stored pinky loadpoint transform: %s at x=%.3f, y=%.3f, z=%.3f", 
                       frame_name.c_str(),
                       transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(node_->get_logger(), "Failed to get pinky loadpoint transform for %s: %s", frame_name.c_str(), ex.what());
        }
    }
    
    // Also try to store additional frames like front_frame and pinky_loadpoint
    std::vector<std::string> additional_frames = {
        pinky_namespace + "/front_frame",
        pinky_namespace + "/pinky_loadpoint"
    };
    
    for (const std::string& frame_name : additional_frames) {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link", frame_name, tf2::TimePointZero, std::chrono::seconds(1));
            
            stored_pinky_transforms_[frame_name] = transform;
            successful_stores++;
            
            RCLCPP_INFO(node_->get_logger(), "Stored additional pinky transform: %s at x=%.3f, y=%.3f, z=%.3f", 
                       frame_name.c_str(),
                       transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_DEBUG(node_->get_logger(), "Optional pinky transform %s not available: %s", frame_name.c_str(), ex.what());
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Successfully stored %d pinky transforms for tag ID %d", successful_stores, tag_id);
    return successful_stores > 0;
}

void TagManager::clearAllStoredPinkyTransforms()
{
    stored_pinky_transforms_.clear();
    RCLCPP_INFO(node_->get_logger(), "Cleared all stored pinky transforms");
}

const std::map<int, geometry_msgs::msg::TransformStamped>& TagManager::getAllStoredTagTransforms() const
{
    return stored_tag_transforms_;
}

void TagManager::detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (!is_collecting_detections_) {
        return;  // Not collecting, ignore
    }
    
    // Add detected tag IDs to the set
    for (const auto& detection : msg->detections) {
        detected_tag_ids_.insert(detection.id);
    }
}

} // namespace jetcobot_picker
