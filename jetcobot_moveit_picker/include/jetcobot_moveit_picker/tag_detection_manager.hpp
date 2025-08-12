#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "constants.hpp"
#include <set>
#include <map>
#include <chrono>

namespace jetcobot_picker {

class TagDetectionManager {
public:
    explicit TagDetectionManager(rclcpp::Node::SharedPtr node);
    
    bool collectDetectedTagsAndAcquireTransforms(double collection_time_seconds);
    bool getStoredTagTransform(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform);
    bool updateStoredTagIfVisible(int tag_id);
    bool findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform);
    
    std::set<int> getDetectedTagIds() const;
    void printDetectedTags() const;
    void clearDetectedTags();

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
    
    std::set<int> detected_tag_ids_;
    std::map<int, geometry_msgs::msg::TransformStamped> stored_tag_transforms_;
    bool is_collecting_detections_;
    std::chrono::steady_clock::time_point detection_start_time_;
    
    void detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    bool tryAcquireTransform(int tag_id);
};

} // namespace jetcobot_picker
