#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "tag_picker", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("tag_picker");

  // Create TF2 buffer and listener
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Wait for tf to be available
  RCLCPP_INFO(logger, "Waiting for AprilTag transform...");
  
  geometry_msgs::msg::TransformStamped tag_transform;
  bool tag_found = false;
  
  // Try to find any tag36h11 transform
  for (int tag_id = 0; tag_id < 100 && !tag_found; ++tag_id) {
    std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
    
    try {
      // Wait up to 5 seconds for the transform
      tag_transform = tf_buffer->lookupTransform(
          "link1",  // target frame
          tag_frame,  // source frame  
          tf2::TimePointZero,  // get latest available
          std::chrono::seconds(1));
      
      tag_found = true;
      RCLCPP_INFO(logger, "Found AprilTag: %s", tag_frame.c_str());
      RCLCPP_INFO(logger, "Tag position: x=%.3f, y=%.3f, z=%.3f", 
                  tag_transform.transform.translation.x,
                  tag_transform.transform.translation.y,
                  tag_transform.transform.translation.z);
      break;
    }
    catch (tf2::TransformException &ex) {
      // Continue searching for other tag IDs
      continue;
    }
  }
  
  if (!tag_found) {
    RCLCPP_ERROR(logger, "No AprilTag found! Make sure tag36h11 detection is running.");
    rclcpp::shutdown();
    return 1;
  }

  // Set target pose based on AprilTag position
  auto const target_pose = [&tag_transform] {
    geometry_msgs::msg::Pose msg;
    
    // Position: move slightly above the tag (offset in z-direction)
    msg.position.x = tag_transform.transform.translation.x;
    msg.position.y = tag_transform.transform.translation.y;
    msg.position.z = tag_transform.transform.translation.z - 0.01; // 1cm above tag

    // Orientation: point down towards the tag
    msg.orientation.w = 0.186;
    msg.orientation.x = -0.682;
    msg.orientation.y = 0.682;
    msg.orientation.z = -0.186;
    
    return msg;
  }();
  
  RCLCPP_INFO(logger, "Setting target pose: x=%.3f, y=%.3f, z=%.3f", 
              target_pose.position.x, target_pose.position.y, target_pose.position.z);
              
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    RCLCPP_INFO(logger, "Plan found! Executing motion to AprilTag...");
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Motion completed!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}