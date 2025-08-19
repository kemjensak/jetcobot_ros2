#include "jetcobot_moveit_picker/core/transform_manager.hpp"
#include <cmath>

namespace jetcobot_picker {

TransformManager::TransformManager(rclcpp::Node* node, std::unique_ptr<tf2_ros::Buffer>& tf_buffer)
    : node_(node), tf_buffer_(tf_buffer)
{
}

void TransformManager::initialize()
{
    // Create static transform broadcaster
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

// ============================================================================
// GROUND PROJECTION OPERATIONS
// ============================================================================

void TransformManager::publishGroundProjectedTransforms(int source_tag_id)
{
    // Define the frames to project based on command type
    std::vector<std::pair<std::string, std::string>> frame_mappings; // source frame, target frame
    
    if (source_tag_id == -1) {
        // SCAN_FRONT command - project tags 31, 32, 33 to pinky pinky_bag_projected frames
        frame_mappings = {
            {"tagStandard41h12:31", "pinky1/pinky_bag_projected"},
            {"tagStandard41h12:32", "pinky2/pinky_bag_projected"},
            {"tagStandard41h12:33", "pinky3/pinky_bag_projected"}
        };
    } else if (source_tag_id == 31 || source_tag_id == 32 || source_tag_id == 33) {
        // SCAN_PINKY command - project tag frames to pinky_bag frames
        std::string tag_frame = "tagStandard41h12:" + std::to_string(source_tag_id);
        std::string pinky_namespace = (source_tag_id == 31) ? "pinky1" : 
                                    (source_tag_id == 32) ? "pinky2" : "pinky3";
        
        frame_mappings = {
            {tag_frame, pinky_namespace + "/pinky_bag_projected"}
        };
    } else {
        RCLCPP_WARN(node_->get_logger(), "No ground projection defined for tag ID %d", source_tag_id);
        return;
    }
    
    // Project and publish each frame
    for (const auto& mapping : frame_mappings) {
        const std::string& source_frame = mapping.first;
        const std::string& target_frame = mapping.second;
        
        try {
            // Get the current transform
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("world", source_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
            
            // Create ground-projected version
            auto ground_transform = createGroundProjectedTransform(transform, target_frame);
            
            // Publish as static transform
            publishStaticTransform(ground_transform);
            
            RCLCPP_INFO(node_->get_logger(), "Published ground-projected transform: %s -> %s", 
                       source_frame.c_str(), target_frame.c_str());
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "Could not get transform for %s: %s", source_frame.c_str(), ex.what());
        }
    }
}

void TransformManager::removeStaticTransformsForMissingTags(int source_tag_id, const std::set<int>& detected_tag_ids)
{
    // Define the frames to remove based on command type
    std::vector<std::string> frames_to_check;
    
    if (source_tag_id == -1) {
        // SCAN_FRONT command - check which tags 31, 32, 33 are missing
        std::vector<int> expected_tags = {31, 32, 33};
        std::vector<std::string> pinky_frames = {"pinky1/pinky_bag_projected", "pinky2/pinky_bag_projected", "pinky3/pinky_bag_projected"};

        for (size_t i = 0; i < expected_tags.size(); i++) {
            int tag_id = expected_tags[i];
            const std::string& frame_name = pinky_frames[i];
            
            // Check if this tag was detected
            if (detected_tag_ids.find(tag_id) == detected_tag_ids.end()) {
                // Tag not detected, remove its static transform
                frames_to_check.push_back(frame_name);
            }
        }
    } else if (source_tag_id == 31 || source_tag_id == 32 || source_tag_id == 33) {
        // SCAN_PINKY command - check if the specific tag is missing
        if (detected_tag_ids.find(source_tag_id) == detected_tag_ids.end()) {
            std::string pinky_namespace = (source_tag_id == 31) ? "pinky1" : 
                                        (source_tag_id == 32) ? "pinky2" : "pinky3";
            frames_to_check.push_back(pinky_namespace + "/front_frame");
            frames_to_check.push_back(pinky_namespace + "/pinky_loadpoint");
        }
    }
    
    // Remove static transforms
    for (const std::string& frame_name : frames_to_check) {
        removeStaticTransform(frame_name);
        RCLCPP_INFO(node_->get_logger(), "Removed static transform for missing tag: %s", frame_name.c_str());
    }
    
    if (!frames_to_check.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Removed %zu static transforms for missing tags", frames_to_check.size());
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "No static transforms to remove - all expected tags detected");
    }
}

geometry_msgs::msg::TransformStamped TransformManager::createGroundProjectedTransform(
    const geometry_msgs::msg::TransformStamped& original_transform,
    const std::string& output_frame_id)
{
    // Extract original position and orientation
    const auto& translation = original_transform.transform.translation;
    const auto& rotation = original_transform.transform.rotation;
    
    // Convert quaternion to euler angles
    auto euler = eulerFromQuaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    double yaw = euler[2];
    
    // Create new quaternion with only yaw rotation (roll=0, pitch=0)
    auto new_quat = quaternionFromEuler(0.0, 0.0, yaw);
    
    // Create new transform message
    geometry_msgs::msg::TransformStamped ground_transform;
    ground_transform.header.stamp = node_->get_clock()->now();
    ground_transform.header.frame_id = original_transform.header.frame_id;
    ground_transform.child_frame_id = output_frame_id;
    
    // Keep x, y translation, maintain z (don't force to ground level)
    ground_transform.transform.translation.x = translation.x;
    ground_transform.transform.translation.y = translation.y;
    ground_transform.transform.translation.z = translation.z;
    
    // Set orientation with only yaw
    ground_transform.transform.rotation.x = new_quat[0];
    ground_transform.transform.rotation.y = new_quat[1];
    ground_transform.transform.rotation.z = new_quat[2];
    ground_transform.transform.rotation.w = new_quat[3];
    
    return ground_transform;
}

void TransformManager::clearAllPinkyStaticTransforms()
{
    std::vector<std::string> pinky_frames_to_clear = {
        "pinky1/pinky_bag_projected",
        "pinky2/pinky_bag_projected", 
        "pinky3/pinky_bag_projected",
        "pinky1/front_frame",
        "pinky2/front_frame",
        "pinky3/front_frame",
        "pinky1/pinky_loadpoint",
        "pinky2/pinky_loadpoint",
        "pinky3/pinky_loadpoint"
    };
    
    for (const std::string& frame_name : pinky_frames_to_clear) {
        removeStaticTransform(frame_name);
        RCLCPP_INFO(node_->get_logger(), "Removed static transform: %s", frame_name.c_str());
    }
    
    RCLCPP_INFO(node_->get_logger(), "Cleared all pinky static transforms (%zu frames)", pinky_frames_to_clear.size());
}

void TransformManager::publishStaticTransform(const geometry_msgs::msg::TransformStamped& transform)
{
    static_tf_broadcaster_->sendTransform(transform);
    
    // Track the published frame for potential removal
    published_static_frames_.insert(transform.child_frame_id);
}

void TransformManager::removeStaticTransform(const std::string& frame_id)
{
    if (published_static_frames_.find(frame_id) != published_static_frames_.end()) {
        // Create an "empty" transform with old timestamp to remove the frame
        geometry_msgs::msg::TransformStamped remove_transform;
        remove_transform.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type()); // Very old timestamp
                    remove_transform.header.frame_id = "base_link";
        remove_transform.child_frame_id = frame_id;
        
        // Set identity transform moved far away
        remove_transform.transform.translation.x = 100.0;
        remove_transform.transform.translation.y = 0.0;
        remove_transform.transform.translation.z = 0.0;
        remove_transform.transform.rotation.x = 0.0;
        remove_transform.transform.rotation.y = 0.0;
        remove_transform.transform.rotation.z = 0.0;
        remove_transform.transform.rotation.w = 1.0;
        
        // Publish the old transform to effectively remove it
        static_tf_broadcaster_->sendTransform(remove_transform);
        
        // Remove from our tracking set
        published_static_frames_.erase(frame_id);
    }
}

std::vector<double> TransformManager::eulerFromQuaternion(double x, double y, double z, double w)
{
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    double pitch;
    if (std::abs(sinp) >= 1) {
        pitch = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
    } else {
        pitch = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return {roll, pitch, yaw};
}

std::vector<double> TransformManager::quaternionFromEuler(double roll, double pitch, double yaw)
{
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    
    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;
    
    return {x, y, z, w};
}

} // namespace jetcobot_picker
