#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class TagPicker : public rclcpp::Node
{
public:
    TagPicker() : Node("tag_picker"),
                  tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
                  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // Create gripper command publisher
        gripper_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_command", 10);
        
        RCLCPP_INFO(get_logger(), "TagPicker initialized successfully");
    }

    bool execute()
    {
        // Initialize MoveGroupInterface after the object is fully constructed
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        move_group_interface_->setMaxAccelerationScalingFactor(0.5);
        move_group_interface_->setMaxVelocityScalingFactor(1.0);
        move_group_interface_->setPlanningTime(15.0);  // Set planning time to 15 seconds
        move_group_interface_->setNumPlanningAttempts(30);
        
        // Find AprilTag
        geometry_msgs::msg::TransformStamped tag_transform;
        if (!findAprilTag(tag_transform)) {
            RCLCPP_ERROR(get_logger(), "No AprilTag found! Make sure tag36h11 detection is running.");
            return false;
        }

        // Move to approach position
        if (!moveToApproachPosition(tag_transform)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to approach position");
            return false;
        }

        // Re-acquire tag transform for better accuracy
        geometry_msgs::msg::TransformStamped updated_tag_transform;
        if (!reacquireTagTransform(updated_tag_transform)) {
            RCLCPP_WARN(get_logger(), "Lost AprilTag after approach! Using original transform.");
            updated_tag_transform = tag_transform;
        }

        // Execute pick and place sequence
        return executePickAndPlace(updated_tag_transform);
    }

private:
    // Member variables
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;

    // Constants
    static constexpr double APPROACH_HEIGHT = 0.1;  // 10cm above tag
    static constexpr double PICK_HEIGHT = -0.01;    // 1cm above tag
    static constexpr double LIFT_HEIGHT = 0.10;     // 6cm lift
    static constexpr double EEF_STEP = 0.01;
    static constexpr double MIN_PATH_FRACTION = 0.8;
    static constexpr int GRIPPER_CLOSE_DELAY_MS = 3000;
    static constexpr int STABILIZE_DELAY_MS = 500;
    static constexpr int OPERATION_DELAY_MS = 1000;

    bool findAprilTag(geometry_msgs::msg::TransformStamped& tag_transform)
    {
        RCLCPP_INFO(get_logger(), "Searching for AprilTag...");
        
        // Try to find any tag36h11 transform
        for (int tag_id = 0; tag_id < 100; ++tag_id) {
            std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
            
            try {
                tag_transform = tf_buffer_->lookupTransform(
                    "link1",  // target frame
                    tag_frame,  // source frame  
                    tf2::TimePointZero,  // get latest available
                    std::chrono::seconds(1));
                
                RCLCPP_INFO(get_logger(), "Found AprilTag: %s", tag_frame.c_str());
                RCLCPP_INFO(get_logger(), "Tag position: x=%.3f, y=%.3f, z=%.3f", 
                           tag_transform.transform.translation.x,
                           tag_transform.transform.translation.y,
                           tag_transform.transform.translation.z);
                return true;
            }
            catch (tf2::TransformException &ex) {
                continue;
            }
        }
        return false;
    }

    geometry_msgs::msg::Pose calculateAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset)
    {
        geometry_msgs::msg::Pose pose;
        
        // Position
        pose.position.x = tag_transform.transform.translation.x;
        pose.position.y = tag_transform.transform.translation.y;
        pose.position.z = tag_transform.transform.translation.z + z_offset;

        // Extract tag's orientation for Y-axis alignment
        tf2::Quaternion tag_quat(
            tag_transform.transform.rotation.x,
            tag_transform.transform.rotation.y,
            tag_transform.transform.rotation.z,
            tag_transform.transform.rotation.w
        );
        
        // Get tag's Y-axis direction
        tf2::Vector3 tag_y_axis(0, 1, 0);
        tag_y_axis = tf2::quatRotate(tag_quat, tag_y_axis);
        
        // Create TCP orientation aligned with tag's Y-axis
        tf2::Vector3 tcp_z_axis(0, 0, -1);  // Point down
        tf2::Vector3 tcp_y_axis = tag_y_axis;  // Align with tag's Y-axis
        tf2::Vector3 tcp_x_axis = tcp_y_axis.cross(tcp_z_axis);
        tcp_x_axis.normalize();
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

    bool moveToApproachPosition(const geometry_msgs::msg::TransformStamped& tag_transform)
    {
        auto approach_pose = calculateAlignedPose(tag_transform, APPROACH_HEIGHT);
        
        RCLCPP_INFO(get_logger(), "Moving to approach position (%.1fcm above): x=%.3f, y=%.3f, z=%.3f", 
                   APPROACH_HEIGHT * 100, approach_pose.position.x, 
                   approach_pose.position.y, approach_pose.position.z);
                   
        move_group_interface_->setPoseTarget(approach_pose, "TCP");

        // Plan and execute approach motion
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        bool success = static_cast<bool>(move_group_interface_->plan(approach_plan));
        
        if (success) {
            RCLCPP_INFO(get_logger(), "Approach plan found! Executing...");
            move_group_interface_->execute(approach_plan);
            RCLCPP_INFO(get_logger(), "Approach motion completed!");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
            return true;
        }
        
        return false;
    }

    bool reacquireTagTransform(geometry_msgs::msg::TransformStamped& updated_tag_transform)
    {
        RCLCPP_INFO(get_logger(), "Re-acquiring AprilTag transform from approach position...");
        
        // Try to find the same tag again
        for (int tag_id = 0; tag_id < 100; ++tag_id) {
            std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
            
            try {
                updated_tag_transform = tf_buffer_->lookupTransform(
                    "link1", tag_frame, tf2::TimePointZero, std::chrono::seconds(1));
                
                RCLCPP_INFO(get_logger(), "Updated AprilTag: %s", tag_frame.c_str());
                RCLCPP_INFO(get_logger(), "Updated tag position: x=%.3f, y=%.3f, z=%.3f", 
                           updated_tag_transform.transform.translation.x,
                           updated_tag_transform.transform.translation.y,
                           updated_tag_transform.transform.translation.z);
                return true;
            }
            catch (tf2::TransformException &ex) {
                continue;
            }
        }
        return false;
    }

    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description)
    {
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_interface_->computeCartesianPath(waypoints, EEF_STEP, trajectory);
        
        if (fraction > MIN_PATH_FRACTION) {
            RCLCPP_INFO(get_logger(), "%s (path fraction: %.2f)", description.c_str(), fraction);
            move_group_interface_->execute(trajectory);
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to plan %s! Cartesian path fraction: %.2f", description.c_str(), fraction);
            return false;
        }
    }

    void controlGripper(bool close)
    {
        auto gripper_msg = std_msgs::msg::Bool();
        gripper_msg.data = close;
        gripper_pub_->publish(gripper_msg);
        
        if (close) {
            RCLCPP_INFO(get_logger(), "Closing gripper...");
            std::this_thread::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_DELAY_MS));
        } else {
            RCLCPP_INFO(get_logger(), "Opening gripper...");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
        }
    }

    bool executePickAndPlace(const geometry_msgs::msg::TransformStamped& tag_transform)
    {
        // Calculate final target pose
        auto final_target_pose = calculateAlignedPose(tag_transform, PICK_HEIGHT);
        
        // Move to final position using Cartesian path
        std::vector<geometry_msgs::msg::Pose> approach_waypoints{final_target_pose};
        if (!executeCartesianPath(approach_waypoints, "final approach to tag")) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Final aligned motion completed!");
        std::this_thread::sleep_for(std::chrono::milliseconds(STABILIZE_DELAY_MS));
        
        // Close gripper
        controlGripper(true);

        // Lift object
        auto lift_pose = final_target_pose;
        lift_pose.position.z += LIFT_HEIGHT;
        
        std::vector<geometry_msgs::msg::Pose> lift_waypoints{lift_pose};
        if (!executeCartesianPath(lift_waypoints, "lifting object")) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Object lifted successfully!");
        std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
        
        // Move back down to original position
        std::vector<geometry_msgs::msg::Pose> down_waypoints{final_target_pose};
        if (!executeCartesianPath(down_waypoints, "returning to original position")) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Returned to original position!");
        std::this_thread::sleep_for(std::chrono::milliseconds(STABILIZE_DELAY_MS));
        
        // Open gripper
        controlGripper(false);
        
        RCLCPP_INFO(get_logger(), "Pick and place operation completed successfully!");
        return true;
    }
};

int main(int argc, char* argv[])
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create and execute TagPicker
    auto tag_picker = std::make_shared<TagPicker>();
    
    bool success = tag_picker->execute();
    
    // Shutdown ROS
    rclcpp::shutdown();
    return success ? 0 : 1;
}