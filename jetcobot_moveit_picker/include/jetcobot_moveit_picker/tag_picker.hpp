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
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
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
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

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
    // ============================================================================
    // CONSTANTS
    // ============================================================================
    struct MovementConstants {
        static constexpr double APPROACH_HEIGHT = 0.05;    // 5cm above tag
        static constexpr double CAM_HEIGHT = 0.09;         // 9cm above TCP
        static constexpr double PICK_HEIGHT = -0.01;       // 1cm below tag surface
        static constexpr double LIFT_HEIGHT = 0.04;        // 3cm lift
        static constexpr double PLACE_HEIGHT = 0.025;      // 2.5cm above tag for placing
        static constexpr double EEF_STEP = 0.001;          // End effector step size
        static constexpr double MIN_PATH_FRACTION = 0.5;   // Minimum path fraction for Cartesian planning
        
        // Safety distances
        static constexpr double MIN_DISTANCE_TO_BASE = 0.001;  // 1mm minimum distance to base for calculations
    };
    
    struct TimingConstants {
        static constexpr int GRIPPER_CLOSE_DELAY_MS = 1000;    // Delay for gripper closing
        static constexpr int STABILIZE_DELAY_MS = 500;         // Stabilization delay
        static constexpr int OPERATION_DELAY_MS = 2000;        // General operation delay
        static constexpr double TAG_COLLECTION_TIME = 1.0;     // Tag detection collection time in seconds
    };
    
    struct GripperPositions {
        static constexpr int FULLY_OPEN = 100;        // Fully open position
        static constexpr int PICKING_POSITION = 20;   // Position for picking objects
        static constexpr int HOLDING_POSITION = 80;   // Position for holding/releasing objects
        static constexpr int FULLY_CLOSED = 0;        // Fully closed position
    };
    
    struct RotationAngles {
        // X-axis rotation angles for multiple approach attempts (in degrees)
        static constexpr int APPROACH_ANGLE_COUNT = 6;
        static inline const std::vector<int> APPROACH_ANGLES = {0, 10, 20, 30, 40, -10};
    };

    // ============================================================================
    // MEMBER VARIABLES
    // ============================================================================
    
    // Core MoveIt and ROS interfaces
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
    rclcpp_action::Server<PickerAction>::SharedPtr action_server_;
    
    // TF broadcasters
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    
    // Data storage
    std::map<int, geometry_msgs::msg::TransformStamped> stored_tag_transforms_;
    std::map<std::string, geometry_msgs::msg::TransformStamped> stored_pinky_transforms_;
    std::set<int> detected_tag_ids_;
    std::chrono::steady_clock::time_point detection_start_time_;
    bool is_collecting_detections_;
    
    // Track published static transforms for removal
    std::set<std::string> published_static_frames_;

    // ============================================================================
    // ACTION SERVER CALLBACKS
    // ============================================================================
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
     * @brief Get stored pinky transform by TF name
     * @param tf_name TF frame name to retrieve
     * @param pinky_transform Output transform
     * @return true if transform found, false otherwise
     */
    bool getStoredPinkyTransform(const std::string& tf_name, geometry_msgs::msg::TransformStamped& pinky_transform);

    /**
     * @brief Store pinky loadpoint transforms
     * @param tag_id Associated tag ID for reference
     * @return true if any transforms acquired, false otherwise
     */
    bool storePinkyLoadpointTransforms(int tag_id);

    /**
     * @brief Publish ground-projected transforms for scan commands
     * @brief Creates ground-projected (roll=0, pitch=0) static TF frames
     * @param source_tag_id Tag ID to determine which frames to project (-1 for SCAN_FRONT)
     */
    void publishGroundProjectedTransforms(int source_tag_id);

    /**
     * @brief Remove static transforms for missing tags
     * @param source_tag_id Tag ID to determine which frames to remove (-1 for SCAN_FRONT)
     */
    void removeStaticTransformsForMissingTags(int source_tag_id);

    /**
     * @brief Create ground-projected transform that removes roll and pitch
     * @param original_transform Original transform to project
     * @param output_frame_id Output frame name
     * @return Ground-projected transform
     */
    geometry_msgs::msg::TransformStamped createGroundProjectedTransform(
        const geometry_msgs::msg::TransformStamped& original_transform, 
        const std::string& output_frame_id);

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
     */
    void removeCollisionObjectsForMissingTags(int source_tag_id);

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
        const std::vector<double>& dimensions);

    // ============================================================================
    // TAG DETECTION AND MANAGEMENT
    // ============================================================================
    
    /**
     * @brief Collect detected tags and acquire their transforms
     * @param collection_time_seconds Time to collect detections
     * @return true if any transforms acquired, false otherwise
     */
    bool collectDetectedTagsAndAcquireTransforms(double collection_time_seconds = 1.0);

    /**
     * @brief Get detected tag IDs
     * @return Set of detected tag IDs
     */
    std::set<int> getDetectedTagIds() const;

    /**
     * @brief Print detected tags to console
     */
    void printDetectedTags() const;

    /**
     * @brief Update stored tag transform if visible from current position
     * @param tag_id Tag ID to update
     * @return true if update successful, false otherwise
     */
    bool updateStoredTagIfVisible(int tag_id);

    // ============================================================================
    // ROBOT MOVEMENT FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Publish collision objects for detected tags
     */
    void PublishBoxCollisionObject();

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
     * @brief Move robot to specified configuration
     * @param config_name Name of the configuration
     * @return true if successful, false otherwise
     */
    bool moveToConfiguration(const std::string& config_name);

    /**
     * @brief Move to position to reacquire tag
     * @param tag_transform Tag transform
     * @param tag_id Tag ID (optional, for pose update)
     * @param angle_index Index of angle from RotationAngles::APPROACH_ANGLES to use (default: 0 for 0 degrees)
     * @return true if successful, false otherwise
     */
    bool moveToReacquireTagPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id = -1, int angle_index = 0);

    /**
     * @brief Execute Cartesian path with given waypoints
     * @param waypoints Vector of poses for path
     * @param description Description for logging
     * @return true if successful, false otherwise
     */
    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description);

    // ============================================================================
    // POSE CALCULATION UTILITIES
    // ============================================================================
    
    /**
     * @brief Calculate single base-aligned pose for tag interaction
     * @param tag_transform Tag transform
     * @param z_offset Height offset from tag
     * @return Calculated pose aligned with robot base direction
     */
    geometry_msgs::msg::Pose calculateBaseAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset);

    /**
     * @brief Calculate multiple base-aligned poses with X-axis rotations
     * @param tag_transform Tag transform
     * @param z_offset Height offset from tag
     * @return Vector of calculated poses with different X-axis rotations (0°, -10°, -20°, -30°, -40°)
     */
    std::vector<geometry_msgs::msg::Pose> calculateBaseAlignedPoses(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset);
    
    // ============================================================================
    // COMMON MOVEMENT PATTERNS
    // ============================================================================
    
    /**
     * @brief Execute stabilized movement with timing delay
     * @param waypoints Poses to execute
     * @param description Movement description for logging
     * @return true if successful, false otherwise
     */
    bool executeStabilizedMovement(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description);
    
    /**
     * @brief Execute lift movement from current position
     * @param lift_height Height to lift in meters
     * @return true if successful, false otherwise
     */
    bool executeLiftMovement(double lift_height);

    // ============================================================================
    // HARDWARE CONTROL
    // ============================================================================
    
    /**
     * @brief Control gripper position with predefined positions
     * @param close_value Gripper value (use GripperPositions constants)
     */
    void controlGripper(int close_value);
    
    /**
     * @brief Open gripper to holding position (for object release)
     */
    void openGripperToHoldingPosition() { controlGripper(GripperPositions::HOLDING_POSITION); }
    
    /**
     * @brief Close gripper to picking position (for object grasping)
     */
    void closeGripperToPicking() { controlGripper(GripperPositions::PICKING_POSITION); }
    
    /**
     * @brief Fully open gripper
     */
    void openGripperFully() { controlGripper(GripperPositions::FULLY_OPEN); }
    
    /**
     * @brief Fully close gripper
     */
    void closeGripperFully() { controlGripper(GripperPositions::FULLY_CLOSED); }

    /**
     * @brief Find specific AprilTag by ID
     * @param tag_id Tag ID to find
     * @param tag_transform Output transform
     * @return true if found, false otherwise
     */
    bool findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform);

    // ============================================================================
    // MAIN PICK AND PLACE OPERATIONS
    // ============================================================================
    
    /**
     * @brief Execute pick operation for specified tag
     * @param tag_id Tag ID to pick from
     * @return true if successful, false otherwise
     */
    bool executePick(int tag_id);

    /**
     * @brief Execute place operation at specified tag location
     * @param target_tag_id Tag ID to place at
     * @param source_tag_id Source tag ID being placed (for pose update)
     * @return true if successful, false otherwise
     */
    bool executePlace(int target_tag_id, int source_tag_id);

    /**
     * @brief Execute place operation at specified TF frame location
     * @param target_tf_name TF frame name to place at
     * @param source_tag_id ID of the tag being placed (for pose updating)
     * @return true if successful, false otherwise
     */
    bool executePlace(const std::string& target_tf_name, int source_tag_id);

    // ============================================================================
    // COMMAND HANDLERS
    // ============================================================================
    
    /**
     * @brief Handle HOME command - move robot to ready position
     */
    bool handleHomeCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN command - scan for tags from current position
     */
    bool handleScanCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN_FRONT command - move to front scan position and scan
     */
    bool handleScanFrontCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN_LEFT command - move to left scan position and scan
     */
    bool handleScanLeftCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN_RIGHT command - move to right scan position and scan
     */
    bool handleScanRightCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle SCAN_PINKY command - approach specific tag and store tag + pinky loadpoint poses
     * @param goal_handle Action goal handle
     */
    bool handleScanPinkyCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle CLEAR_PINKY command - remove all pinky-related static transforms and collision objects
     * @param goal_handle Action goal handle
     */
    bool handleClearPinkyCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);

    /**
     * @brief Handle PICK_AND_PLACE command - execute complete pick and place operation
     * @param goal_handle Action goal handle
     */
    bool handlePickAndPlaceCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle);
};

#endif // TAG_PICKER_HPP
