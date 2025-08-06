#ifndef TAG_PICKER_HPP
#define TAG_PICKER_HPP

#include <memory>
#include <chrono>
#include <thread>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <jetcobot_interfaces/action/picker_action.hpp>

class TagPicker : public rclcpp::Node
{
public:
    using PickerAction = jetcobot_interfaces::action::PickerAction;
    using GoalHandlePickerAction = rclcpp_action::ServerGoalHandle<PickerAction>;

    /**
     * @brief Constructor for TagPicker
     */
    TagPicker();

    /**
     * @brief Main execution function
     * @return true if execution successful, false otherwise
     */
    bool execute();

private:
    // Constants
    static constexpr double APPROACH_HEIGHT = 0.05;  // 5cm above tag
    static constexpr double CAM_HEIGHT = 0.08;       // 8cm above TCP
    static constexpr double PICK_HEIGHT = -0.01;     // 1cm below tag surface
    static constexpr double LIFT_HEIGHT = 0.07;      // 7cm lift
    static constexpr double PLACE_HEIGHT = 0.025;    // 2.5cm above tag for placing
    static constexpr double EEF_STEP = 0.005;        // End effector step size
    static constexpr double MIN_PATH_FRACTION = 0.4; // Minimum path fraction for Cartesian planning
    static constexpr int GRIPPER_CLOSE_DELAY_MS = 1500;
    static constexpr int STABILIZE_DELAY_MS = 500;
    static constexpr int OPERATION_DELAY_MS = 1500;

    // Member variables
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
    rclcpp_action::Server<PickerAction>::SharedPtr action_server_;
    
    // Storage for tag transforms
    std::map<int, geometry_msgs::msg::TransformStamped> stored_tag_transforms_;
    
    // Detection-related storage
    std::set<int> detected_tag_ids_;
    std::chrono::steady_clock::time_point detection_start_time_;
    bool is_collecting_detections_;

    // Callback functions
    /**
     * @brief Goal callback for action server
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickerAction::Goal> goal);

    /**
     * @brief Cancel callback for action server
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Accept callback for action server
     */
    void handle_accepted(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Execute the action goal
     */
    void execute_goal(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Callback for AprilTag detection messages
     * @param msg Detection array message
     */
    void detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    // Tag management functions
    /**
     * @brief Get stored transform for a specific tag
     * @param tag_id Tag ID to retrieve
     * @param tag_transform Output transform
     * @return true if transform found, false otherwise
     */
    bool getStoredTagTransform(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform);

    /**
     * @brief Collect detected tags and acquire their transforms
     * @param collection_time_seconds Time to collect detections
     * @return true if any transforms acquired, false otherwise
     */
    bool collectDetectedTagsAndAcquireTransforms(double collection_time_seconds = 1.0);

    /**
     * @brief Get set of detected tag IDs
     * @return Set of detected tag IDs
     */
    std::set<int> getDetectedTagIds() const;

    /**
     * @brief Print all detected tags to console
     */
    void printDetectedTags() const;

    /**
     * @brief Update stored tag transform if visible from current position
     * @param tag_id Tag ID to update
     * @return true if update successful, false otherwise
     */
    bool updateStoredTagIfVisible(int tag_id);

    // Movement functions
    /**
     * @brief Move robot to home position
     * @return true if successful, false otherwise
     */
    bool moveToHome();

    /**
     * @brief Move to position to reacquire tag
     * @param tag_transform Tag transform
     * @param tag_id Tag ID (optional, for pose update)
     * @return true if successful, false otherwise
     */
    bool moveToReacquireTagPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id = -1);

    /**
     * @brief Execute Cartesian path with given waypoints
     * @param waypoints Vector of poses for path
     * @param description Description for logging
     * @return true if successful, false otherwise
     */
    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description);

    // Utility functions
    /**
     * @brief Calculate base-aligned pose for tag interaction
     * @param tag_transform Tag transform
     * @param z_offset Height offset from tag
     * @return Calculated pose
     */
    geometry_msgs::msg::Pose calculateBaseAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset);

    /**
     * @brief Control gripper position
     * @param close_value Gripper value (0=close, 100=open)
     */
    void controlGripper(int close_value);

    /**
     * @brief Find specific AprilTag by ID
     * @param tag_id Tag ID to find
     * @param tag_transform Output transform
     * @return true if found, false otherwise
     */
    bool findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform);

    // Main operation functions
    /**
     * @brief Execute pick operation for specified tag
     * @param tag_id Tag ID to pick from
     * @return true if successful, false otherwise
     */
    bool executePick(int tag_id);

    /**
     * @brief Execute place operation at specified tag location
     * @param target_tag_id Tag ID to place at
     * @return true if successful, false otherwise
     */
    bool executePlace(int target_tag_id);

    /**
     * @brief Execute place operation at specified TF frame location
     * @param target_tf_name TF frame name to place at
     * @return true if successful, false otherwise
     */
    bool executePlace(const std::string& target_tf_name);

    // Command handling functions
    /**
     * @brief Handle HOME command
     */
    bool handleHomeCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN command
     */
    bool handleScanCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle PICK_AND_PLACE command
     * @param goal_handle Action goal handle
     */
    bool handlePickAndPlaceCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
};

#endif // TAG_PICKER_HPP
