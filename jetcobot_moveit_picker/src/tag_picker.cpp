#include <memory>
#include <chrono>
#include <thread>
#include <map>

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
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
        move_group_interface_->setMaxAccelerationScalingFactor(1.0);
        move_group_interface_->setMaxVelocityScalingFactor(1.0);
        move_group_interface_->setPlanningTime(15.0);  // Set planning time to 15 seconds
        move_group_interface_->setNumPlanningAttempts(100);
        
        // Move to home position first
        if (!moveToHome()) {
            RCLCPP_WARN(get_logger(), "Failed to move to home position, continuing anyway...");
        }

        controlGripper(false);  // Close gripper to ensure it's ready

        // Scan and store all visible AprilTags before starting operations
        if (!scanAndStoreAllTags()) {
            RCLCPP_ERROR(get_logger(), "No AprilTags found! Make sure tag36h11 detection is running.");
            return false;
        }
        
        // Example usage of new pick and place functions
        // Pick from tag ID 6 and place at tag ID 9
        
        // Pick operation
        // int pick_tag_id = 9;
        // if (!executePick(pick_tag_id)) {
        //     RCLCPP_ERROR(get_logger(), "Pick operation failed for tag ID %d", pick_tag_id);
        //     return false;
        // }
        
        // // Place operation
        // int place_tag_id = 6;
        // if (!executePlace(place_tag_id)) {
        //     RCLCPP_ERROR(get_logger(), "Place operation failed for tag ID %d", place_tag_id);
        //     return false;
        // }

        // // Return to home position
        // if (!moveToHome()) {
        //     RCLCPP_WARN(get_logger(), "Failed to return to home position");
        // }

        int pick_tag_id = 4;
        if (!executePick(pick_tag_id)) {
            RCLCPP_ERROR(get_logger(), "Pick operation failed for tag ID %d", pick_tag_id);
            return false;
        }

        int place_tag_id = 9;
        if (!executePlace(place_tag_id)) {
            RCLCPP_ERROR(get_logger(), "Place operation failed for tag ID %d", place_tag_id);
            return false;
        }

        
        // Return to home position
        if (!moveToHome()) {
            RCLCPP_WARN(get_logger(), "Failed to return to home position");
        }
        
        RCLCPP_INFO(get_logger(), "Complete pick and place operation successful!");
        return true;
    }

private:
    // Member variables
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
    
    // Storage for tag transforms
    std::map<int, geometry_msgs::msg::TransformStamped> stored_tag_transforms_;

    // Constants
    static constexpr double APPROACH_HEIGHT = 0.09;  // 9cm above tag
    static constexpr double CAM_HEIGHT = 0.07;  // 7cm above TCP
    static constexpr double PICK_HEIGHT = -0.02;    // 0 above tag
    static constexpr double LIFT_HEIGHT = 0.10;     // 10cm lift
    static constexpr double PLACE_HEIGHT = 0.02;   // 2cm above tag for placing
    static constexpr double EEF_STEP = 0.01;
    static constexpr double MIN_PATH_FRACTION = 0.8;
    static constexpr int GRIPPER_CLOSE_DELAY_MS = 1500;
    static constexpr int STABILIZE_DELAY_MS = 1000;
    static constexpr int OPERATION_DELAY_MS = 2000;

    bool scanAndStoreAllTags()
    {
        RCLCPP_INFO(get_logger(), "Scanning and storing all visible AprilTags...");
        
        stored_tag_transforms_.clear();
        int tags_found = 0;
        
        // Try to find all visible tags
        for (int tag_id = 0; tag_id < 10; ++tag_id) {
            std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
            
            try {
                geometry_msgs::msg::TransformStamped tag_transform = tf_buffer_->lookupTransform(
                    "base_link",  // target frame
                    tag_frame,  // source frame  
                    tf2::TimePointZero,  // get latest available
                    std::chrono::seconds(1));
                
                // Store the transform
                stored_tag_transforms_[tag_id] = tag_transform;
                tags_found++;
                
                RCLCPP_INFO(get_logger(), "Stored AprilTag ID %d: x=%.3f, y=%.3f, z=%.3f", 
                           tag_id,
                           tag_transform.transform.translation.x,
                           tag_transform.transform.translation.y,
                           tag_transform.transform.translation.z);
            }
            catch (tf2::TransformException &ex) {
                continue;
            }
        }
        
        RCLCPP_INFO(get_logger(), "Found and stored %d AprilTags", tags_found);
        return tags_found > 0;
    }

    bool getStoredTagTransform(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform)
    {
        auto it = stored_tag_transforms_.find(tag_id);
        if (it != stored_tag_transforms_.end()) {
            tag_transform = it->second;
            RCLCPP_INFO(get_logger(), "Retrieved stored transform for tag ID %d", tag_id);
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "No stored transform found for tag ID %d", tag_id);
            return false;
        }
    }

    bool findAprilTag(geometry_msgs::msg::TransformStamped& tag_transform)
    {
        RCLCPP_INFO(get_logger(), "Searching for AprilTag...");
        
        // Try to find any tag36h11 transform
        for (int tag_id = 0; tag_id < 100; ++tag_id) {
            std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
            
            try {
                tag_transform = tf_buffer_->lookupTransform(
                    "base_link",  // target frame
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

    bool moveToHome()
    {
        RCLCPP_INFO(get_logger(), "Moving to home position...");
        
        // Get available named targets for debugging
        std::vector<std::string> named_targets = move_group_interface_->getNamedTargets();
        RCLCPP_INFO(get_logger(), "Available named targets:");
        for (const auto& target : named_targets) {
            RCLCPP_INFO(get_logger(), "  - %s", target.c_str());
        }
        
        // Try common home position names
        std::vector<std::string> state_groups = {"home", "ready_to_see", "ready_to_grasp"};

        move_group_interface_->setNamedTarget("ready_to_see");
        
        moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        bool success = static_cast<bool>(move_group_interface_->plan(home_plan));
        
        if (success) {
            RCLCPP_INFO(get_logger(), "Home plan found! Executing...");
            move_group_interface_->execute(home_plan);
            RCLCPP_INFO(get_logger(), "Successfully moved to home position!");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
            return true;
        }
        
        RCLCPP_ERROR(get_logger(), "Failed to plan move to home position!");
        return false;
    }

    geometry_msgs::msg::Pose calculateBaseAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset)
    {
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
        
        // Direction toward base (normalized)
        double distance_to_base = sqrt(tag_x * tag_x + tag_y * tag_y);
        tf2::Vector3 base_direction(-tag_x / distance_to_base, -tag_y / distance_to_base, 0);
        
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
        
        RCLCPP_INFO(get_logger(), "Base direction: (%.3f, %.3f), Best alignment axis: Tag %s (dot product: %.3f)", 
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

    bool updateStoredTagIfVisible(int tag_id)
    {
        // return false;
        RCLCPP_INFO(get_logger(), "Checking if tag ID %d is visible for pose update...", tag_id);
        
        std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
        
        try {
            geometry_msgs::msg::TransformStamped updated_transform = tf_buffer_->lookupTransform(
                "base_link",  // target frame
                tag_frame,  // source frame  
                tf2::TimePointZero,  // get latest available
                std::chrono::seconds(1));
            
            // Update the stored transform with the new, more accurate data
            stored_tag_transforms_[tag_id] = updated_transform;
            
            RCLCPP_INFO(get_logger(), "Updated stored transform for tag ID %d: x=%.3f, y=%.3f, z=%.3f", 
                       tag_id,
                       updated_transform.transform.translation.x,
                       updated_transform.transform.translation.y,
                       updated_transform.transform.translation.z);
            return true;
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Tag ID %d not visible from current position: %s", tag_id, ex.what());
            return false;
        }
    }

    bool moveToApproachPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id = -1)
    {
        // Use look-at pose for approach position to orient TCP toward tag center
        auto approach_pose = calculateBaseAlignedPose(tag_transform, APPROACH_HEIGHT);

        RCLCPP_INFO(get_logger(), "Moving to approach position (%.1fcm above) with look-at orientation: x=%.3f, y=%.3f, z=%.3f",
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
        }
        
        return false;
    }

    bool moveToReacquireTagPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id = -1)
    {
        // Use look-at pose for approach position to orient TCP toward tag center
        auto tag_pose = calculateBaseAlignedPose(tag_transform, CAM_HEIGHT + APPROACH_HEIGHT);

        RCLCPP_INFO(get_logger(), "Moving to tag position (%.1fcm above) with look-at orientation: x=%.3f, y=%.3f, z=%.3f",
                   CAM_HEIGHT * 100, tag_pose.position.x,
                   tag_pose.position.y, tag_pose.position.z);

        move_group_interface_->setPoseTarget(tag_pose, "jetcocam");

        // Plan and execute approach motion
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        bool success = static_cast<bool>(move_group_interface_->plan(approach_plan));
        
        if (success) {
            RCLCPP_INFO(get_logger(), "Approach plan found! Executing...");
            move_group_interface_->execute(approach_plan);
            RCLCPP_INFO(get_logger(), "Approach motion completed!");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
            
            // Try to update the stored tag pose if the tag is visible from the new position
            if (tag_id >= 0) {
                updateStoredTagIfVisible(tag_id);
            }
            
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
                    "base_link", tag_frame, tf2::TimePointZero, std::chrono::seconds(1));
                
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

    bool findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform)
    {
        std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
        
        try {
            tag_transform = tf_buffer_->lookupTransform(
                "base_link",  // target frame
                tag_frame,  // source frame  
                tf2::TimePointZero,  // get latest available
                std::chrono::seconds(1));
            
            RCLCPP_INFO(get_logger(), "Found specific AprilTag: %s", tag_frame.c_str());
            RCLCPP_INFO(get_logger(), "Tag position: x=%.3f, y=%.3f, z=%.3f", 
                       tag_transform.transform.translation.x,
                       tag_transform.transform.translation.y,
                       tag_transform.transform.translation.z);
            return true;
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "Failed to find AprilTag ID %d: %s", tag_id, ex.what());
            return false;
        }
    }

    bool executePick(int tag_id)
    {
        RCLCPP_INFO(get_logger(), "Starting pick operation for tag ID: %d", tag_id);
        
        // Get the stored tag transform
        geometry_msgs::msg::TransformStamped tag_transform;
        if (!getStoredTagTransform(tag_id, tag_transform)) {
            RCLCPP_ERROR(get_logger(), "Cannot find stored transform for tag ID %d", tag_id);
            return false;
        }

        // Move to tag position first
        if (!moveToReacquireTagPosition(tag_transform, tag_id)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to tag position for tag %d", tag_id);
            return false;
        }

        // Move to approach position
        // if (!moveToApproachPosition(tag_transform, tag_id)) {
        //     RCLCPP_ERROR(get_logger(), "Failed to move to approach position for tag %d", tag_id);
        //     return false;
        // }
        
        // Get the potentially updated tag transform after approach
        geometry_msgs::msg::TransformStamped updated_tag_transform;
        if (getStoredTagTransform(tag_id, updated_tag_transform)) {
            tag_transform = updated_tag_transform;
            RCLCPP_INFO(get_logger(), "Using updated tag transform for final approach");
        }
        
        // Calculate final target pose using (potentially updated) stored transform
        auto final_target_pose = calculateBaseAlignedPose(tag_transform, PICK_HEIGHT);
        
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
        
        RCLCPP_INFO(get_logger(), "Pick operation completed for tag ID: %d", tag_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
        
        return true;
    }

    bool executePlace(int target_tag_id)
    {
        RCLCPP_INFO(get_logger(), "Starting place operation at tag ID: %d", target_tag_id);
        
        // Get the stored target tag transform
        geometry_msgs::msg::TransformStamped target_tag_transform;
        if (!getStoredTagTransform(target_tag_id, target_tag_transform)) {
            RCLCPP_ERROR(get_logger(), "Cannot find stored transform for target tag ID %d", target_tag_id);
            return false;
        }

        // Move to target tag position first
        if (!moveToReacquireTagPosition(target_tag_transform, target_tag_id)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to target tag position for tag %d", target_tag_id);
            return false;
        }
        
        // Move to approach position above target location
        // if (!moveToApproachPosition(target_tag_transform, target_tag_id)) {
        //     RCLCPP_ERROR(get_logger(), "Failed to move to approach position for target tag %d", target_tag_id);
        //     return false;
        // }
        
        // Get the potentially updated target tag transform after approach
        geometry_msgs::msg::TransformStamped updated_target_tag_transform;
        if (getStoredTagTransform(target_tag_id, updated_target_tag_transform)) {
            target_tag_transform = updated_target_tag_transform;
            RCLCPP_INFO(get_logger(), "Using updated target tag transform for placement");
        }
        
        // Calculate placement pose using (potentially updated) stored transform
        auto place_pose = calculateBaseAlignedPose(target_tag_transform, PLACE_HEIGHT);
        
        // Move down to placement position
        std::vector<geometry_msgs::msg::Pose> place_waypoints{place_pose};
        if (!executeCartesianPath(place_waypoints, "moving to placement position")) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Reached placement position!");
        std::this_thread::sleep_for(std::chrono::milliseconds(STABILIZE_DELAY_MS));
        
        // Open gripper to release object
        controlGripper(false);
        
        // Lift up slightly after placing
        auto lift_after_place_pose = place_pose;
        lift_after_place_pose.position.z += APPROACH_HEIGHT;
        
        // std::vector<geometry_msgs::msg::Pose> lift_after_place_waypoints{lift_after_place_pose};
        // if (!executeCartesianPath(lift_after_place_waypoints, "lifting after placement")) {
        //     return false;
        // }
        
        RCLCPP_INFO(get_logger(), "Place operation completed at tag ID: %d", target_tag_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
        
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