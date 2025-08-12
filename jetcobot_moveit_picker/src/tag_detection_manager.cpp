#include "jetcobot_moveit_picker/tag_detection_manager.hpp"
#include <thread>

namespace jetcobot_picker {

TagDetectionManager::TagDetectionManager(rclcpp::Node::SharedPtr node) 
    : node_(node), is_collecting_detections_(false) {
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    detection_sub_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", 10,
        std::bind(&TagDetectionManager::detectionCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "TagDetectionManager initialized");
}

void TagDetectionManager::detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    if (!is_collecting_detections_) {
        return;
    }
    
    // Add detected tag IDs to our collection
    for (const auto& detection : msg->detections) {
        detected_tag_ids_.insert(detection.id);
        RCLCPP_INFO(node_->get_logger(), "Detected tag ID: %d", detection.id);
    }
}

bool TagDetectionManager::collectDetectedTagsAndAcquireTransforms(double collection_time_seconds) {
    RCLCPP_INFO(node_->get_logger(), "Starting tag detection collection for %.1f seconds...", collection_time_seconds);
    
    // Clear previous detections and start collection
    detected_tag_ids_.clear();
    is_collecting_detections_ = true;
    detection_start_time_ = std::chrono::steady_clock::now();
    
    // Collect detections for the specified time
    auto collection_duration = std::chrono::duration<double>(collection_time_seconds);
    while (rclcpp::ok()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = current_time - detection_start_time_;
        
        if (elapsed >= collection_duration) {
            break;
        }
        
        // Just sleep and let the main executor handle callbacks
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Stop collecting detections
    is_collecting_detections_ = false;
    
    // Now acquire transforms for all detected tags
    int transforms_acquired = 0;
    for (int tag_id : detected_tag_ids_) {
        if (tryAcquireTransform(tag_id)) {
            transforms_acquired++;
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Successfully acquired transforms for %d out of %zu detected tags", 
               transforms_acquired, detected_tag_ids_.size());
    
    return transforms_acquired > 0;
}

bool TagDetectionManager::tryAcquireTransform(int tag_id) {
    std::vector<std::string> possible_frame_formats = {
        "tagStandard41h12:" + std::to_string(tag_id),
        "tag_" + std::to_string(tag_id),
        "apriltag_" + std::to_string(tag_id)
    };
    
    for (const auto& frame_format : possible_frame_formats) {
        try {
            geometry_msgs::msg::TransformStamped tag_transform = tf_buffer_->lookupTransform(
                "base_link",  // target frame
                frame_format,  // source frame  
                tf2::TimePointZero,  // get latest available
                std::chrono::seconds(1));
            
            // Store the transform
            stored_tag_transforms_[tag_id] = tag_transform;
            
            RCLCPP_INFO(node_->get_logger(), "Acquired transform for tag ID %d (%s): x=%.3f, y=%.3f, z=%.3f", 
                       tag_id, frame_format.c_str(),
                       tag_transform.transform.translation.x,
                       tag_transform.transform.translation.y,
                       tag_transform.transform.translation.z);
            return true;
        }
        catch (tf2::TransformException &ex) {
            continue;  // Try next format
        }
    }
    
    RCLCPP_WARN(node_->get_logger(), "Could not acquire transform for detected tag ID %d", tag_id);
    return false;
}

bool TagDetectionManager::getStoredTagTransform(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform) {
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

bool TagDetectionManager::updateStoredTagIfVisible(int tag_id) {
    RCLCPP_INFO(node_->get_logger(), "Checking if tag ID %d is visible for pose update...", tag_id);
    
    std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
    
    try {
        geometry_msgs::msg::TransformStamped updated_transform = tf_buffer_->lookupTransform(
            "base_link",  // target frame
            tag_frame,  // source frame  
            tf2::TimePointZero,  // get latest available
            std::chrono::seconds(1));
        
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

bool TagDetectionManager::findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform) {
    std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
    
    try {
        tag_transform = tf_buffer_->lookupTransform(
            "base_link",  // target frame
            tag_frame,  // source frame  
            tf2::TimePointZero,  // get latest available
            std::chrono::seconds(1));
        
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

std::set<int> TagDetectionManager::getDetectedTagIds() const {
    return detected_tag_ids_;
}

void TagDetectionManager::printDetectedTags() const {
    RCLCPP_INFO(node_->get_logger(), "Currently detected tag IDs (%zu total):", detected_tag_ids_.size());
    for (int tag_id : detected_tag_ids_) {
        RCLCPP_INFO(node_->get_logger(), "  - Tag ID: %d", tag_id);
    }
}

void TagDetectionManager::clearDetectedTags() {
    detected_tag_ids_.clear();
    stored_tag_transforms_.clear();
}

} // namespace jetcobot_picker
