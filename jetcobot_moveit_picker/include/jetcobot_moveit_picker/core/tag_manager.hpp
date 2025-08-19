#ifndef TAG_MANAGER_HPP
#define TAG_MANAGER_HPP

#include <memory>
#include <map>
#include <set>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

namespace jetcobot_picker {

/**
 * @brief Manages AprilTag detection, tracking, and transform storage
 */
class TagManager 
{
public:
    /**
     * @brief Constructor
     * @param node ROS node pointer for logging and subscriptions
     * @param tf_buffer TF buffer for transform lookups
     */
    TagManager(rclcpp::Node* node, std::unique_ptr<tf2_ros::Buffer>& tf_buffer);

    /**
     * @brief Initialize tag detection subscription
     */
    void initialize();

    // ============================================================================
    // TAG DETECTION AND MANAGEMENT
    // ============================================================================

    /**
     * @brief Collect detected tags and acquire their transforms
     * @param collection_time_seconds Time to collect detections
     * @return true if any transforms acquired, false otherwise
     */
    bool collectDetectedTagsAndAcquireTransforms(double collection_time_seconds = 1.0);

    /**
     * @brief Get detected tag IDs
     * @return Set of detected tag IDs
     */
    std::set<int> getDetectedTagIds() const;

    /**
     * @brief Print detected tags to console
     */
    void printDetectedTags() const;

    /**
     * @brief Update stored tag transform if visible from current position
     * @param tag_id Tag ID to update
     * @return true if update successful, false otherwise
     */
    bool updateStoredTagIfVisible(int tag_id);

    /**
     * @brief Find specific AprilTag by ID
     * @param tag_id Tag ID to find
     * @param tag_transform Output transform
     * @return true if found, false otherwise
     */
    bool findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform);

    // ============================================================================
    // TRANSFORM STORAGE AND RETRIEVAL
    // ============================================================================

    /**
     * @brief Get stored transform for a specific tag
     * @param tag_id Tag ID to retrieve
     * @param tag_transform Output transform
     * @return true if transform found, false otherwise
     */
    bool getStoredTagTransform(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform);

    /**
     * @brief Store a tag transform
     * @param tag_id Tag ID
     * @param transform Transform to store
     */
    void storeTagTransform(int tag_id, const geometry_msgs::msg::TransformStamped& transform);

    /**
     * @brief Remove stored tag transform
     * @param tag_id Tag ID to remove
     */
    void removeStoredTagTransform(int tag_id);

    /**
     * @brief Clear all stored tag transforms
     */
    void clearAllStoredTagTransforms();

    /**
     * @brief Get stored pinky transform by TF name
     * @param tf_name TF frame name to retrieve
     * @param pinky_transform Output transform
     * @return true if transform found, false otherwise
     */
    bool getStoredPinkyTransform(const std::string& tf_name, geometry_msgs::msg::TransformStamped& pinky_transform);

    /**
     * @brief Store pinky loadpoint transforms
     * @param tag_id Associated tag ID for reference
     * @return true if any transforms acquired, false otherwise
     */
    bool storePinkyLoadpointTransforms(int tag_id);

    /**
     * @brief Clear all stored pinky transforms
     */
    void clearAllStoredPinkyTransforms();

    /**
     * @brief Get all stored tag transforms
     * @return Map of tag ID to transform
     */
    const std::map<int, geometry_msgs::msg::TransformStamped>& getAllStoredTagTransforms() const;

private:
    rclcpp::Node* node_;
    std::unique_ptr<tf2_ros::Buffer>& tf_buffer_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;

    // Data storage
    std::map<int, geometry_msgs::msg::TransformStamped> stored_tag_transforms_;
    std::map<std::string, geometry_msgs::msg::TransformStamped> stored_pinky_transforms_;
    std::set<int> detected_tag_ids_;
    std::chrono::steady_clock::time_point detection_start_time_;
    bool is_collecting_detections_;

    /**
     * @brief Callback for AprilTag detection messages
     * @param msg Detection array message
     */
    void detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
};

} // namespace jetcobot_picker

#endif // TAG_MANAGER_HPP
