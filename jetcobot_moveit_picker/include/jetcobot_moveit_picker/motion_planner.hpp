#pragma once

// Suppress deprecated header warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#pragma GCC diagnostic pop

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include "constants.hpp"
#include <vector>
#include <string>

namespace jetcobot_picker {

struct RotationAngles {
    static const std::vector<int> APPROACH_ANGLES;
};

class MotionPlanner {
public:
    explicit MotionPlanner(rclcpp::Node::SharedPtr node);
    
    bool initialize();
    bool moveToConfiguration(const std::string& config_name);
    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description);
    bool executeStabilizedMovement(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description);
    bool executeLiftMovement(double lift_height);
    
    geometry_msgs::msg::Pose getCurrentPose(const std::string& end_effector_link = "TCP");
    
    geometry_msgs::msg::Pose calculateBaseAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset);
    std::vector<geometry_msgs::msg::Pose> calculateBaseAlignedPoses(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset);
    
    bool moveToReacquireTagPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id = -1);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    
    void logAvailableNamedTargets();
};

} // namespace jetcobot_picker
