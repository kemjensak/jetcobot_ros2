#ifndef TRANSFORM_MANAGER_HPP
#define TRANSFORM_MANAGER_HPP

#include <memory>
#include <set>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace jetcobot_picker {

/**
 * @brief Manages TF frame publishing and ground projection operations
 */
class TransformManager 
{
public:
    /**
     * @brief Constructor
     * @param node ROS node pointer for logging and broadcasting
     * @param tf_buffer TF buffer for transform lookups
     */
    TransformManager(rclcpp::Node* node, std::unique_ptr<tf2_ros::Buffer>& tf_buffer);

    /**
     * @brief Initialize static transform broadcaster
     */
    void initialize();

    // ============================================================================
    // GROUND PROJECTION OPERATIONS
    // ============================================================================

    /**
     * @brief Publish ground-projected transforms for scan commands
     * @brief Creates ground-projected (roll=0, pitch=0) static TF frames
     * @param source_tag_id Tag ID to determine which frames to project (-1 for SCAN_FRONT)
     */
    void publishGroundProjectedTransforms(int source_tag_id);

    /**
     * @brief Remove static transforms for missing tags
     * @param source_tag_id Tag ID to determine which frames to remove (-1 for SCAN_FRONT)
     * @param detected_tag_ids Set of currently detected tag IDs
     */
    void removeStaticTransformsForMissingTags(int source_tag_id, const std::set<int>& detected_tag_ids);

    /**
     * @brief Create ground-projected transform that removes roll and pitch
     * @param original_transform Original transform to project
     * @param output_frame_id Output frame name
     * @return Ground-projected transform
     */
    geometry_msgs::msg::TransformStamped createGroundProjectedTransform(
        const geometry_msgs::msg::TransformStamped& original_transform, 
        const std::string& output_frame_id
    );

    /**
     * @brief Clear all pinky-related static transforms
     */
    void clearAllPinkyStaticTransforms();

    /**
     * @brief Publish a static transform
     * @param transform Transform to publish
     */
    void publishStaticTransform(const geometry_msgs::msg::TransformStamped& transform);

    /**
     * @brief Remove a static transform by publishing with old timestamp
     * @param frame_id Child frame ID to remove
     */
    void removeStaticTransform(const std::string& frame_id);

private:
    rclcpp::Node* node_;
    std::unique_ptr<tf2_ros::Buffer>& tf_buffer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    
    // Track published static transforms for removal
    std::set<std::string> published_static_frames_;

    // Utility functions for transform calculations
    /**
     * @brief Convert quaternion to euler angles
     * @param x Quaternion x component
     * @param y Quaternion y component  
     * @param z Quaternion z component
     * @param w Quaternion w component
     * @return Euler angles [roll, pitch, yaw]
     */
    std::vector<double> eulerFromQuaternion(double x, double y, double z, double w);

    /**
     * @brief Convert euler angles to quaternion
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians  
     * @param yaw Yaw angle in radians
     * @return Quaternion [x, y, z, w]
     */
    std::vector<double> quaternionFromEuler(double roll, double pitch, double yaw);
};

} // namespace jetcobot_picker

#endif // TRANSFORM_MANAGER_HPP
