#include "jetcobot_moveit_picker/core/collision_manager.hpp"
#include <cmath>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace jetcobot_picker {

CollisionManager::CollisionManager(
    rclcpp::Node* node,
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
    std::unique_ptr<tf2_ros::Buffer>& tf_buffer)
    : node_(node), planning_scene_interface_(planning_scene_interface), tf_buffer_(tf_buffer)
{
}

// ============================================================================
// COLLISION OBJECT MANAGEMENT
// ============================================================================

void CollisionManager::createCollisionObjectsAtPinkyBagPoses(int source_tag_id)
{
    // Define a structure to hold the three frame names
    struct PinkyFrameMapping {
        std::string bag_frame;
        std::string collision_id;
        std::string base_frame;
    };
    
    // Define the frames to create collisions based on command type
    std::vector<PinkyFrameMapping> collision_mappings;
    
    if (source_tag_id == -1) {
        // SCAN_FRONT command - create collisions for pinky1/2/3 pinky_bag frames
        collision_mappings = {
            {"pinky1/pinky_bag_projected", "pinky1_bag_collision", "pinky1/base_link"},
            {"pinky2/pinky_bag_projected", "pinky2_bag_collision", "pinky2/base_link"},
            {"pinky3/pinky_bag_projected", "pinky3_bag_collision", "pinky3/base_link"}
        };
    } else if (source_tag_id == 31 || source_tag_id == 32 || source_tag_id == 33) {
        // SCAN_PINKY command - create collision for specific pinky bag
        std::string pinky_namespace = (source_tag_id == 31) ? "pinky1" : 
                                    (source_tag_id == 32) ? "pinky2" : "pinky3";
        
        collision_mappings = {
            {pinky_namespace + "/pinky_bag_projected", pinky_namespace + "_bag_collision", pinky_namespace + "/base_link"}
        };
    } else {
        RCLCPP_WARN(node_->get_logger(), "No collision objects defined for tag ID %d", source_tag_id);
        return;
    }
    
    // Create collision objects at each frame
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    for (const auto& mapping : collision_mappings) {
        const std::string& frame_name = mapping.bag_frame;
        const std::string& collision_id = mapping.collision_id;
        const std::string& base_frame_name = mapping.base_frame;

        try {
            // Get the current transform for the pinky bag frame
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("world", frame_name, tf2::TimePointZero, tf2::durationFromSec(1.0));
            
            // Convert transform to pose with ground projection
            geometry_msgs::msg::Pose pose = createGroundProjectedPose(transform);
            pose.position.z -= 0.003;  // Slight adjustment for bag position
            
            // Create box collision object (representing pinky bag dimensions)
            std::vector<double> dimensions = {0.075, 0.115, 0.0025};
            auto collision_object = createBoxCollisionObject(collision_id, pose, dimensions);
            collision_objects.push_back(collision_object);

            // Get base frame transform
            geometry_msgs::msg::TransformStamped transform_base = 
                tf_buffer_->lookupTransform("robot_base_link", base_frame_name, tf2::TimePointZero, tf2::durationFromSec(1.0));
            
            // Create base collision object at offset
            geometry_msgs::msg::Pose base_pose;
            base_pose.position.x = transform_base.transform.translation.x - 0.005;
            base_pose.position.y = transform_base.transform.translation.y;
            base_pose.position.z = transform_base.transform.translation.z + 0.105/2 - 0.03;
            base_pose.orientation = transform_base.transform.rotation;

            std::string base_collision_id = collision_id + "_base_collision";
            std::vector<double> base_dimensions = {0.125, 0.12, 0.105};
            auto base_collision_object = createBoxCollisionObject(base_collision_id, base_pose, base_dimensions);
            collision_objects.push_back(base_collision_object);
            
            // Create lidar collision object
            geometry_msgs::msg::Pose lidar_pose;
            lidar_pose.position.x = transform_base.transform.translation.x;
            lidar_pose.position.y = transform_base.transform.translation.y;
            lidar_pose.position.z = base_pose.position.z + 0.105/2 + 0.04/2;
            lidar_pose.orientation = transform_base.transform.rotation;

            std::string lidar_collision_id = collision_id + "_lidar_collision";
            std::vector<double> lidar_dimensions = {0.05, 0.05, 0.04};
            auto lidar_collision_object = createBoxCollisionObject(lidar_collision_id, lidar_pose, lidar_dimensions);
            collision_objects.push_back(lidar_collision_object);

            RCLCPP_INFO(node_->get_logger(), "Created collision objects: %s, %s, and %s at frame %s", 
                       collision_id.c_str(), base_collision_id.c_str(), lidar_collision_id.c_str(), frame_name.c_str());
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "Could not get transform for %s to create collision: %s", 
                       frame_name.c_str(), ex.what());
        }
    }
    
    // Add all collision objects to the planning scene
    if (!collision_objects.empty()) {
        planning_scene_interface_->addCollisionObjects(collision_objects);
        RCLCPP_INFO(node_->get_logger(), "Added %zu collision objects to planning scene", collision_objects.size());
    }
}

void CollisionManager::removeCollisionObjectsForMissingTags(int source_tag_id, const std::set<int>& detected_tag_ids)
{
    std::vector<std::string> collision_ids_to_remove;
    
    if (source_tag_id == -1) {
        // SCAN_FRONT command - check which tags 31, 32, 33 are missing
        std::vector<int> expected_tags = {31, 32, 33};
        std::vector<std::string> pinky_namespaces = {"pinky1", "pinky2", "pinky3"};
        
        for (size_t i = 0; i < expected_tags.size(); i++) {
            int tag_id = expected_tags[i];
            const std::string& pinky_namespace = pinky_namespaces[i];
            
            // Check if this tag was detected
            if (detected_tag_ids.find(tag_id) == detected_tag_ids.end()) {
                // Tag not detected, mark collision objects for removal
                collision_ids_to_remove.push_back(pinky_namespace + "_bag_collision");
                collision_ids_to_remove.push_back(pinky_namespace + "_bag_collision_base_collision");
                collision_ids_to_remove.push_back(pinky_namespace + "_bag_collision_lidar_collision");
                RCLCPP_INFO(node_->get_logger(), "Tag %d not detected, will remove collision objects for %s", 
                           tag_id, pinky_namespace.c_str());
            }
        }
    } else if (source_tag_id == 31 || source_tag_id == 32 || source_tag_id == 33) {
        // SCAN_PINKY command - remove collision for specific tag (called when tag not visible)
        std::string pinky_namespace = (source_tag_id == 31) ? "pinky1" : 
                                    (source_tag_id == 32) ? "pinky2" : "pinky3";
        collision_ids_to_remove.push_back(pinky_namespace + "_bag_collision");
        collision_ids_to_remove.push_back(pinky_namespace + "_bag_collision_base_collision");
        collision_ids_to_remove.push_back(pinky_namespace + "_bag_collision_lidar_collision");
        RCLCPP_INFO(node_->get_logger(), "Tag %d not visible after approach, removing collision objects for %s", 
                   source_tag_id, pinky_namespace.c_str());
    }
    
    // Remove collision objects from planning scene
    removeCollisionObjects(collision_ids_to_remove);
}

void CollisionManager::publishBoxCollisionObjects(const std::map<int, geometry_msgs::msg::TransformStamped>& stored_tag_transforms)
{
    RCLCPP_INFO(node_->get_logger(), "Publishing collision objects for %zu stored tags", stored_tag_transforms.size());
    
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    for (const auto& pair : stored_tag_transforms) {
        int tag_id = pair.first;
        const geometry_msgs::msg::TransformStamped& transform = pair.second;
        
        // Create ground-projected pose for collision object
        geometry_msgs::msg::Pose pose = createGroundProjectedPose(transform);
        
        // Create collision object for each tag
        std::string object_id = "tag_" + std::to_string(tag_id) + "_collision";
        std::vector<double> dimensions = {0.04, 0.04, 0.002}; // AprilTag dimensions
        
        auto collision_object = createBoxCollisionObject(object_id, pose, dimensions);
        collision_objects.push_back(collision_object);
        
        RCLCPP_INFO(node_->get_logger(), "Created collision object for tag %d at x=%.3f, y=%.3f, z=%.3f", 
                   tag_id, pose.position.x, pose.position.y, pose.position.z);
    }
    
    if (!collision_objects.empty()) {
        planning_scene_interface_->addCollisionObjects(collision_objects);
        RCLCPP_INFO(node_->get_logger(), "Added %zu tag collision objects to planning scene", collision_objects.size());
    }
}

void CollisionManager::attachBoxToGripper(int tag_id)
{
    std::string object_id = "tag_" + std::to_string(tag_id) + "_collision";
    
    moveit_msgs::msg::AttachedCollisionObject attach_object;
    attach_object.link_name = "TCP";
    attach_object.object.id = object_id;
    attach_object.object.operation = attach_object.object.ADD;
    
    // Define the box primitive for the attached object
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.04; // length
    primitive.dimensions[primitive.BOX_Y] = 0.04; // width
    primitive.dimensions[primitive.BOX_Z] = 0.002; // height (very thin for AprilTag)
    
    // Set relative pose (attached at TCP)
    geometry_msgs::msg::Pose relative_pose;
    relative_pose.position.x = 0.0;
    relative_pose.position.y = 0.0;
    relative_pose.position.z = -0.01; // Slightly below TCP
    relative_pose.orientation.w = 1.0;
    
    attach_object.object.primitives.push_back(primitive);
    attach_object.object.primitive_poses.push_back(relative_pose);
    attach_object.object.header.frame_id = "TCP";
    
    planning_scene_interface_->applyAttachedCollisionObject(attach_object);
    
    RCLCPP_INFO(node_->get_logger(), "Attached box collision object %s to TCP", object_id.c_str());
}

void CollisionManager::detachBoxFromGripper(int tag_id)
{
    std::string object_id = "tag_" + std::to_string(tag_id) + "_collision";
    
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.link_name = "TCP";
    detach_object.object.id = object_id;
    detach_object.object.operation = detach_object.object.REMOVE;
    
    planning_scene_interface_->applyAttachedCollisionObject(detach_object);
    
    RCLCPP_INFO(node_->get_logger(), "Detached box collision object %s from TCP", object_id.c_str());
}

moveit_msgs::msg::CollisionObject CollisionManager::createBoxCollisionObject(
    const std::string& object_id,
    const geometry_msgs::msg::Pose& pose,
    const std::vector<double>& dimensions)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.header.stamp = node_->get_clock()->now();
    collision_object.id = object_id;
    
    // Define the box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = dimensions[0]; // length
    primitive.dimensions[primitive.BOX_Y] = dimensions[1]; // width  
    primitive.dimensions[primitive.BOX_Z] = dimensions[2]; // height
    
    // Set pose and primitive
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;
    
    return collision_object;
}

void CollisionManager::removeCollisionObjects(const std::vector<std::string>& collision_ids)
{
    if (!collision_ids.empty()) {
        planning_scene_interface_->removeCollisionObjects(collision_ids);
        RCLCPP_INFO(node_->get_logger(), "Removed %zu collision objects from planning scene", collision_ids.size());
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "No collision objects to remove");
    }
}

void CollisionManager::clearAllPinkyCollisionObjects()
{
    std::vector<std::string> collision_ids_to_remove = {
        "pinky1_bag_collision",
        "pinky2_bag_collision", 
        "pinky3_bag_collision",
        "pinky1_bag_collision_base_collision",
        "pinky2_bag_collision_base_collision",
        "pinky3_bag_collision_base_collision",
        "pinky1_bag_collision_lidar_collision",
        "pinky2_bag_collision_lidar_collision",
        "pinky3_bag_collision_lidar_collision"
    };
    
    removeCollisionObjects(collision_ids_to_remove);
    RCLCPP_INFO(node_->get_logger(), "Cleared all pinky collision objects");
}

geometry_msgs::msg::Pose CollisionManager::createGroundProjectedPose(const geometry_msgs::msg::TransformStamped& transform)
{
    // Extract original position and orientation
    const auto& translation = transform.transform.translation;
    const auto& rotation = transform.transform.rotation;
    
    // Convert quaternion to euler angles
    auto euler = eulerFromQuaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    double yaw = euler[2];
    
    // Create new quaternion with only yaw rotation (roll=0, pitch=0) - parallel to ground
    auto new_quat = quaternionFromEuler(0.0, 0.0, yaw);
    
    // Create pose with ground-projected orientation
    geometry_msgs::msg::Pose pose;
    pose.position.x = translation.x;
    pose.position.y = translation.y;
    pose.position.z = translation.z;
    pose.orientation.x = new_quat[0];
    pose.orientation.y = new_quat[1];
    pose.orientation.z = new_quat[2];
    pose.orientation.w = new_quat[3];
    
    return pose;
}

std::vector<double> CollisionManager::eulerFromQuaternion(double x, double y, double z, double w)
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

std::vector<double> CollisionManager::quaternionFromEuler(double roll, double pitch, double yaw)
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
