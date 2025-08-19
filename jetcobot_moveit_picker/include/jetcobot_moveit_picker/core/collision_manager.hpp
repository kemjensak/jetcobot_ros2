#ifndef COLLISION_MANAGER_HPP
#define COLLISION_MANAGER_HPP

#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>

namespace jetcobot_picker {

/**
 * @brief Manages MoveIt collision objects for safe robot operation
 */
class CollisionManager 
{
public:
    /**
     * @brief Constructor
     * @param node ROS node pointer for logging
     * @param planning_scene_interface MoveIt planning scene interface
     * @param tf_buffer TF buffer for transform lookups
     */
    CollisionManager(
        rclcpp::Node* node,
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
        std::unique_ptr<tf2_ros::Buffer>& tf_buffer
    );

    // ============================================================================
    // COLLISION OBJECT MANAGEMENT
    // ============================================================================

    /**
     * @brief Create collision objects at pinky bag poses after scan commands
     * @param source_tag_id Tag ID to determine which collision objects to create (-1 for SCAN_FRONT)
     */
    void createCollisionObjectsAtPinkyBagPoses(int source_tag_id);

    /**
     * @brief Remove collision objects when corresponding tags are no longer detected
     * @param source_tag_id Tag ID to determine which collision objects to remove (-1 for SCAN_FRONT)
     * @param detected_tag_ids Set of currently detected tag IDs
     */
    void removeCollisionObjectsForMissingTags(int source_tag_id, const std::set<int>& detected_tag_ids);

    /**
     * @brief Publish collision objects for detected tags
     * @param stored_tag_transforms Map of stored tag transforms
     */
    void publishBoxCollisionObjects(const std::map<int, geometry_msgs::msg::TransformStamped>& stored_tag_transforms);

    /**
     * @brief Attach a box collision object to the gripper
     * @param tag_id Tag ID of the box to attach
     */
    void attachBoxToGripper(int tag_id);

    /**
     * @brief Detach a box collision object from the gripper
     * @param tag_id Tag ID of the box to detach
     */
    void detachBoxFromGripper(int tag_id);

    /**
     * @brief Create a box collision object at specified pose
     * @param object_id Unique ID for the collision object
     * @param pose Pose where to place the collision object
     * @param dimensions Box dimensions [x, y, z] in meters
     * @return CollisionObject message
     */
    moveit_msgs::msg::CollisionObject createBoxCollisionObject(
        const std::string& object_id,
        const geometry_msgs::msg::Pose& pose,
        const std::vector<double>& dimensions
    );

    /**
     * @brief Remove all collision objects with specified IDs
     * @param collision_ids Vector of collision object IDs to remove
     */
    void removeCollisionObjects(const std::vector<std::string>& collision_ids);

    /**
     * @brief Remove all pinky-related collision objects
     */
    void clearAllPinkyCollisionObjects();

    /**
     * @brief Create ground-projected pose from transform (parallel to ground plane)
     * @param transform Input transform to project
     * @return Ground-projected pose with only yaw rotation
     */
    geometry_msgs::msg::Pose createGroundProjectedPose(const geometry_msgs::msg::TransformStamped& transform);

private:
    rclcpp::Node* node_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<tf2_ros::Buffer>& tf_buffer_;

    // Utility functions for collision object creation
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

#endif // COLLISION_MANAGER_HPP
