#pragma once

#include "gripper_controller.hpp"
#include "tag_detection_manager.hpp"
#include "motion_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace jetcobot_picker {

class PickPlaceExecutor {
public:
    PickPlaceExecutor(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<GripperController> gripper_controller,
        std::shared_ptr<TagDetectionManager> tag_detection_manager,
        std::shared_ptr<MotionPlanner> motion_planner
    );
    
    bool executePick(int tag_id);
    bool executePlace(int target_tag_id, int source_tag_id);
    bool executePlace(const std::string& target_tf_name, int source_tag_id);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<GripperController> gripper_controller_;
    std::shared_ptr<TagDetectionManager> tag_detection_manager_;
    std::shared_ptr<MotionPlanner> motion_planner_;
    
    bool performPickSequence(const geometry_msgs::msg::TransformStamped& tag_transform);
    bool performPlaceSequence(const geometry_msgs::msg::TransformStamped& target_transform, int source_tag_id);
};

} // namespace jetcobot_picker
