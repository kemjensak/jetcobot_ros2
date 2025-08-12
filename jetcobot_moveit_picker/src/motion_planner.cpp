#include "jetcobot_moveit_picker/motion_planner.hpp"
#include <thread>

namespace jetcobot_picker {

const std::vector<int> RotationAngles::APPROACH_ANGLES = {0, -15, -30, -45};

MotionPlanner::MotionPlanner(rclcpp::Node::SharedPtr node) : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "MotionPlanner initialized");
}

bool MotionPlanner::initialize() {
    try {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_group");
        move_group_interface_->setMaxAccelerationScalingFactor(0.9);
        move_group_interface_->setMaxVelocityScalingFactor(1.0);
        move_group_interface_->setPlanningTime(10.0);
        move_group_interface_->setNumPlanningAttempts(200);
        
        RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface initialized successfully");
        logAvailableNamedTargets();
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
        return false;
    }
}

void MotionPlanner::logAvailableNamedTargets() {
    std::vector<std::string> named_targets = move_group_interface_->getNamedTargets();
    RCLCPP_INFO(node_->get_logger(), "Available named targets:");
    for (const auto& target : named_targets) {
        RCLCPP_INFO(node_->get_logger(), "  - %s", target.c_str());
    }
}

bool MotionPlanner::moveToConfiguration(const std::string& config_name) {
    RCLCPP_INFO(node_->get_logger(), "Moving to configuration: %s", config_name.c_str());
    
    move_group_interface_->setNamedTarget(config_name);
    moveit::planning_interface::MoveGroupInterface::Plan config_plan;
    bool success = static_cast<bool>(move_group_interface_->plan(config_plan));
    
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Configuration plan found! Executing...");
        move_group_interface_->execute(config_plan);
        RCLCPP_INFO(node_->get_logger(), "Successfully moved to configuration: %s", config_name.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
        return true;
    }
    
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan move to configuration: %s", config_name.c_str());
    return false;
}

bool MotionPlanner::executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface_->computeCartesianPath(waypoints, MovementConstants::EEF_STEP, trajectory);

    if (fraction > MovementConstants::MIN_PATH_FRACTION) {
        RCLCPP_INFO(node_->get_logger(), "%s (path fraction: %.2f)", description.c_str(), fraction);
        move_group_interface_->execute(trajectory);
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to plan %s! Cartesian path fraction: %.2f", description.c_str(), fraction);
        return false;
    }
}

bool MotionPlanner::executeStabilizedMovement(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description) {
    if (!executeCartesianPath(waypoints, description)) {
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Movement completed: %s", description.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::STABILIZE_DELAY_MS));
    return true;
}

bool MotionPlanner::executeLiftMovement(double lift_height) {
    geometry_msgs::msg::Pose current_pose = getCurrentPose("TCP");
    current_pose.position.z += lift_height;
    
    std::vector<geometry_msgs::msg::Pose> lift_waypoints{current_pose};
    return executeStabilizedMovement(lift_waypoints, "lifting movement");
}

geometry_msgs::msg::Pose MotionPlanner::getCurrentPose(const std::string& end_effector_link) {
    return move_group_interface_->getCurrentPose(end_effector_link).pose;
}

geometry_msgs::msg::Pose MotionPlanner::calculateBaseAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset) {
    geometry_msgs::msg::Pose pose;
    
    // Position
    pose.position.x = tag_transform.transform.translation.x;
    pose.position.y = tag_transform.transform.translation.y;
    pose.position.z = tag_transform.transform.translation.z + z_offset;

    // Extract tag's orientation
    tf2::Quaternion tag_quat(
        tag_transform.transform.rotation.x,
        tag_transform.transform.rotation.y,
        tag_transform.transform.rotation.z,
        tag_transform.transform.rotation.w
    );
    
    // Calculate direction vector from tag to robot base (origin)
    double tag_x = tag_transform.transform.translation.x;
    double tag_y = tag_transform.transform.translation.y;
    
    // Direction toward base (normalized) - with safety check for division by zero
    double distance_to_base = sqrt(tag_x * tag_x + tag_y * tag_y);
    tf2::Vector3 base_direction;
    
    if (distance_to_base < MovementConstants::MIN_DISTANCE_TO_BASE) {  // Very close to base
        RCLCPP_WARN(node_->get_logger(), "TF frame very close to robot base (distance: %.6f), using default Y-axis alignment", distance_to_base);
        base_direction = tf2::Vector3(0, 1, 0);  // Default to Y-axis direction
    } else {
        base_direction = tf2::Vector3(-tag_x / distance_to_base, -tag_y / distance_to_base, 0);
    }
    
    // Get tag's 4 principal axes (+X, -X, +Y, -Y)
    tf2::Vector3 tag_axes[4];
    tag_axes[0] = tf2::quatRotate(tag_quat, tf2::Vector3(1, 0, 0));   // +X
    tag_axes[1] = tf2::quatRotate(tag_quat, tf2::Vector3(-1, 0, 0));  // -X
    tag_axes[2] = tf2::quatRotate(tag_quat, tf2::Vector3(0, 1, 0));   // +Y
    tag_axes[3] = tf2::quatRotate(tag_quat, tf2::Vector3(0, -1, 0));  // -Y
    
    // Project all tag axes to XY plane (remove Z component)
    for (int i = 0; i < 4; i++) {
        tag_axes[i].setZ(0);
        tag_axes[i].normalize();
    }
    
    // Find the tag axis that is most aligned with base direction
    double max_dot_product = -1.0;
    int best_axis_index = 0;
    std::string axis_names[4] = {"+X", "-X", "+Y", "-Y"};
    
    for (int i = 0; i < 4; i++) {
        double dot_product = base_direction.dot(tag_axes[i]);
        if (dot_product > max_dot_product) {
            max_dot_product = dot_product;
            best_axis_index = i;
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Base direction: (%.3f, %.3f), Best alignment axis: Tag %s (dot product: %.3f)", 
               base_direction.x(), base_direction.y(), axis_names[best_axis_index].c_str(), max_dot_product);
    
    // Create TCP orientation with base-pointing axis alignment
    tf2::Vector3 tcp_z_axis(0, 0, -1);  // Point down (maintain vertical orientation)
    tf2::Vector3 tcp_y_axis = tag_axes[best_axis_index];  // Use base-pointing tag axis
    tf2::Vector3 tcp_x_axis = tcp_y_axis.cross(tcp_z_axis);
    tcp_x_axis.normalize();
    
    // Recalculate Y-axis to ensure orthogonality
    tcp_y_axis = tcp_z_axis.cross(tcp_x_axis);
    tcp_y_axis.normalize();
    
    // Create rotation matrix and convert to quaternion
    tf2::Matrix3x3 rotation_matrix(
        tcp_x_axis.x(), tcp_y_axis.x(), tcp_z_axis.x(),
        tcp_x_axis.y(), tcp_y_axis.y(), tcp_z_axis.y(),
        tcp_x_axis.z(), tcp_y_axis.z(), tcp_z_axis.z()
    );
    
    tf2::Quaternion tcp_quat;
    rotation_matrix.getRotation(tcp_quat);
    tcp_quat.normalize();
    
    pose.orientation.x = tcp_quat.x();
    pose.orientation.y = tcp_quat.y();
    pose.orientation.z = tcp_quat.z();
    pose.orientation.w = tcp_quat.w();
    
    return pose;
}

std::vector<geometry_msgs::msg::Pose> MotionPlanner::calculateBaseAlignedPoses(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset) {
    std::vector<geometry_msgs::msg::Pose> poses;
    
    // First, get the base aligned pose
    geometry_msgs::msg::Pose base_pose = calculateBaseAlignedPose(tag_transform, z_offset);
    
    // Convert base orientation to quaternion
    tf2::Quaternion base_quat(
        base_pose.orientation.x,
        base_pose.orientation.y,
        base_pose.orientation.z,
        base_pose.orientation.w
    );

    // Generate poses with X-axis rotations using predefined angles
    const auto& angles = RotationAngles::APPROACH_ANGLES;
    
    for (int angle_deg : angles) {
        geometry_msgs::msg::Pose pose;
        
        // Same position for all poses
        pose.position = base_pose.position;
        
        // Create X-axis rotation quaternion
        double angle_rad = angle_deg * M_PI / 180.0;
        tf2::Quaternion x_rotation;
        x_rotation.setRPY(angle_rad, 0, 0);  // Roll around X-axis
        
        // Apply X-axis rotation to the base orientation
        tf2::Quaternion rotated_quat = base_quat * x_rotation;
        rotated_quat.normalize();
        
        // Set the rotated orientation
        pose.orientation.x = rotated_quat.x();
        pose.orientation.y = rotated_quat.y();
        pose.orientation.z = rotated_quat.z();
        pose.orientation.w = rotated_quat.w();
        
        poses.push_back(pose);
        
        RCLCPP_DEBUG(node_->get_logger(), "Generated pose for X-axis rotation %d°: position(%.3f, %.3f, %.3f)", 
                    angle_deg, pose.position.x, pose.position.y, pose.position.z);
    }
    
    RCLCPP_INFO(node_->get_logger(), "Generated %zu poses with X-axis rotations: 0°, -15°, -30°, -45°", poses.size());
    
    return poses;
}

bool MotionPlanner::moveToReacquireTagPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int /* tag_id */) {
    // Use look-at pose for approach position to orient TCP toward tag center
    auto tag_pose = calculateBaseAlignedPose(tag_transform, MovementConstants::CAM_HEIGHT + MovementConstants::APPROACH_HEIGHT);

    RCLCPP_INFO(node_->get_logger(), "Moving to tag position (%.1fcm above) with look-at orientation: x=%.3f, y=%.3f, z=%.3f",
               MovementConstants::CAM_HEIGHT * 100, tag_pose.position.x,
               tag_pose.position.y, tag_pose.position.z);
               
    move_group_interface_->clearPoseTargets();
    move_group_interface_->setPoseTarget(tag_pose, "jetcocam");

    // Plan and execute approach motion
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    bool success = static_cast<bool>(move_group_interface_->plan(approach_plan));
    
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Approach plan found! Executing...");
        move_group_interface_->execute(approach_plan);
        RCLCPP_INFO(node_->get_logger(), "Approach motion completed!");
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
        return true;
    }
    
    return false;
}

} // namespace jetcobot_picker
