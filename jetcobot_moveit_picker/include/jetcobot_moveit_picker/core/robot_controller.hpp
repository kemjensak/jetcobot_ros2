#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

namespace jetcobot_picker {

/**
 * @brief Core robot movement and hardware control functionality
 */
class RobotController 
{
public:
    // Movement constants
    struct MovementConstants {
        static constexpr double APPROACH_HEIGHT = 0.05;    // 5cm above tag
        static constexpr double CAM_HEIGHT = 0.09;         // 9cm above TCP
        static constexpr double PICK_HEIGHT = -0.01;       // 1cm below tag surface
        static constexpr double LIFT_HEIGHT = 0.06;        // 3cm lift
        static constexpr double PLACE_HEIGHT = 0.025;      // 2.5cm above tag for placing
        static constexpr double EEF_STEP = 0.001;          // End effector step size
        static constexpr double MIN_PATH_FRACTION = 0.3;   // Minimum path fraction for Cartesian planning
        static constexpr double MIN_DISTANCE_TO_BASE = 0.001;  // 1mm minimum distance to base for calculations
    };

    // Timing constants
    struct TimingConstants {
        static constexpr int GRIPPER_CLOSE_DELAY_MS = 1000;    // Delay for gripper closing
        static constexpr int STABILIZE_DELAY_MS = 500;         // Stabilization delay
        static constexpr int OPERATION_DELAY_MS = 2000;        // General operation delay
        static constexpr double TAG_COLLECTION_TIME = 1.0;     // Tag detection collection time in seconds
    };

    // Gripper positions
    struct GripperPositions {
        static constexpr int FULLY_OPEN = 100;        // Fully open position
        static constexpr int PICKING_POSITION = 20;   // Position for picking objects
        static constexpr int HOLDING_POSITION = 80;   // Position for holding/releasing objects
        static constexpr int FULLY_CLOSED = 0;        // Fully closed position
    };

    // Rotation angles for multiple approach attempts
    struct RotationAngles {
        static constexpr int APPROACH_ANGLE_COUNT = 6;
        static inline const std::vector<int> APPROACH_ANGLES = {0, 10, 20, 30, 40, -10};
    };

    /**
     * @brief Constructor
     * @param node ROS node pointer for logging and publishing
     * @param move_group MoveIt move group interface
     * @param gripper_pub Gripper command publisher
     */
    RobotController(
        rclcpp::Node* node,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub
    );

    // ============================================================================
    // BASIC MOVEMENT FUNCTIONS
    // ============================================================================

    /**
     * @brief Move robot to specified configuration
     * @param config_name Name of the configuration
     * @return true if successful, false otherwise
     */
    bool moveToConfiguration(const std::string& config_name);

    /**
     * @brief Execute Cartesian path with given waypoints
     * @param waypoints Vector of poses for path
     * @param description Description for logging
     * @return true if successful, false otherwise
     */
    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description);

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

    /**
     * @brief Move to position to reacquire tag
     * @param tag_transform Tag transform
     * @param tag_id Tag ID (optional, for pose update)
     * @param angle_index Index of angle from RotationAngles::APPROACH_ANGLES to use (default: 0 for 0 degrees)
     * @return true if successful, false otherwise
     */
    bool moveToReacquireTagPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id = -1, int angle_index = 0);

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
     * @return Vector of calculated poses with different X-axis rotations
     */
    std::vector<geometry_msgs::msg::Pose> calculateBaseAlignedPoses(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset);

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
     * @brief Get current end-effector pose
     * @param frame_id Frame to get pose relative to (default: "TCP")
     * @return Current pose
     */
    geometry_msgs::msg::Pose getCurrentPose(const std::string& frame_id = "TCP");

private:
    rclcpp::Node* node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;
};

} // namespace jetcobot_picker

#endif // ROBOT_CONTROLLER_HPP
