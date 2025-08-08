#include "jetcobot_moveit_picker/tag_picker.hpp"
#include <thread>

// Constructor implementation
TagPicker::TagPicker() : Node("tag_picker"),
                          tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
                          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
                          is_collecting_detections_(false)
{
    // Create gripper command publisher
    gripper_pub_ = create_publisher<std_msgs::msg::Int32>("/gripper_command", 10);
    
    // Create subscriber for AprilTag detections
    detection_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", 10,
        std::bind(&TagPicker::detectionCallback, this, std::placeholders::_1));
    
    // Create action server
    action_server_ = rclcpp_action::create_server<PickerAction>(
        this,
        "picker_action",
        std::bind(&TagPicker::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TagPicker::handle_cancel, this, std::placeholders::_1),
        std::bind(&TagPicker::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "TagPicker initialized successfully");
}

bool TagPicker::execute()
{
    // Initialize MoveGroupInterface after the object is fully constructed
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    move_group_interface_->setMaxAccelerationScalingFactor(0.9);
    move_group_interface_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_->setPlanningTime(10.0);  // Set planning time to 15 seconds
    move_group_interface_->setNumPlanningAttempts(200);
    
    openGripperToHoldingPosition();
    // controlGripper(100);  // Open gripper fully to start fresh

    RCLCPP_INFO(get_logger(), "TagPicker ready to receive action commands!");
    
    // Keep the node alive to process callbacks
    rclcpp::spin(shared_from_this());
    
    return true;
}

rclcpp_action::GoalResponse TagPicker::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickerAction::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request: command=%s, source_id=%d, target_id=%d", 
               goal->command.c_str(), goal->source_tag_id, goal->target_tag_id);
    
    // Accept all goals for now
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TagPicker::handle_cancel(
    const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TagPicker::handle_accepted(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    // Execute goal in a separate thread
    std::thread{std::bind(&TagPicker::execute_goal, this, goal_handle)}.detach();
}

void TagPicker::execute_goal(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing goal");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PickerAction::Feedback>();
    auto result = std::make_shared<PickerAction::Result>();
    
    bool success = false;
    
    if (goal->command == "HOME") {
        success = handleHomeCommand(goal_handle);
    }
    else if (goal->command == "SCAN") {
        success = handleScanCommand(goal_handle);
    }
    else if (goal->command == "SCAN_FRONT") {
        success = handleScanFrontCommand(goal_handle);
    }
    else if (goal->command == "SCAN_LEFT") {
        success = handleScanLeftCommand(goal_handle);
    }
    else if (goal->command == "SCAN_RIGHT") {
        success = handleScanRightCommand(goal_handle);
    }
    else if (goal->command == "PICK_AND_PLACE") {
        success = handlePickAndPlaceCommand(goal_handle);
    }
    else {
        RCLCPP_ERROR(get_logger(), "Unknown command: %s", goal->command.c_str());
        result->success = false;
        result->error_message = "Unknown command: " + goal->command;
        goal_handle->abort(result);
        return;
    }
    
    // Set final result
    result->success = success;
    if (!success) {
        result->error_message = "Command execution failed";
    }
    
    // Send final feedback
    feedback->current_phase = "completed";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    if (success) {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
    } else {
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "Goal aborted");
    }
}

bool TagPicker::getStoredTagTransform(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform)
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

void TagPicker::detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (!is_collecting_detections_) {
        return;
    }
    
    // Add detected tag IDs to our collection
    for (const auto& detection : msg->detections) {
        detected_tag_ids_.insert(detection.id);
        RCLCPP_INFO(get_logger(), "Detected tag ID: %d", detection.id);
    }
}

bool TagPicker::collectDetectedTagsAndAcquireTransforms(double collection_time_seconds)
{
    RCLCPP_INFO(get_logger(), "Starting tag detection collection for %.1f seconds...", collection_time_seconds);
    
    // Clear previous detections and start collection
    detected_tag_ids_.clear();
    // stored_tag_transforms_.clear();
    is_collecting_detections_ = true;
    detection_start_time_ = std::chrono::steady_clock::now();
    
    // Collect detections for the specified time
    auto collection_duration = std::chrono::duration<double>(collection_time_seconds);
    while (rclcpp::ok()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = current_time - detection_start_time_;
        
        if (elapsed >= collection_duration) {
            break;
        }
        
        // Just sleep and let the main executor handle callbacks
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Stop collecting detections
    is_collecting_detections_ = false;
    
    // Now acquire transforms for all detected tags
    int transforms_acquired = 0;
    for (int tag_id : detected_tag_ids_) {
        std::string tag_frame;
        
        // Try different tag frame formats
        std::vector<std::string> possible_frame_formats = {
            "tagStandard41h12:" + std::to_string(tag_id),
            "tag_" + std::to_string(tag_id),
            "apriltag_" + std::to_string(tag_id)
        };
        
        bool transform_found = false;
        for (const auto& frame_format : possible_frame_formats) {
            try {
                geometry_msgs::msg::TransformStamped tag_transform = tf_buffer_->lookupTransform(
                    "base_link",  // target frame
                    frame_format,  // source frame  
                    tf2::TimePointZero,  // get latest available
                    std::chrono::seconds(1));
                
                // Store the transform
                stored_tag_transforms_[tag_id] = tag_transform;
                transforms_acquired++;
                transform_found = true;
                
                RCLCPP_INFO(get_logger(), "Acquired transform for tag ID %d (%s): x=%.3f, y=%.3f, z=%.3f", 
                           tag_id, frame_format.c_str(),
                           tag_transform.transform.translation.x,
                           tag_transform.transform.translation.y,
                           tag_transform.transform.translation.z);
                break;
            }
            catch (tf2::TransformException &ex) {
                continue;  // Try next format
            }
        }
        
        if (!transform_found) {
            RCLCPP_WARN(get_logger(), "Could not acquire transform for detected tag ID %d", tag_id);
        }
    }
    
    RCLCPP_INFO(get_logger(), "Successfully acquired transforms for %d out of %zu detected tags", 
               transforms_acquired, detected_tag_ids_.size());
    
    return transforms_acquired > 0;
}

std::set<int> TagPicker::getDetectedTagIds() const
{
    return detected_tag_ids_;
}

void TagPicker::printDetectedTags() const
{
    RCLCPP_INFO(get_logger(), "Currently detected tag IDs (%zu total):", detected_tag_ids_.size());
    for (int tag_id : detected_tag_ids_) {
        RCLCPP_INFO(get_logger(), "  - Tag ID: %d", tag_id);
    }
}

bool TagPicker::moveToConfiguration(const std::string& config_name)
{
    RCLCPP_INFO(get_logger(), "Moving to configuration: %s", config_name.c_str());
    
    // Get available named targets for debugging
    std::vector<std::string> named_targets = move_group_interface_->getNamedTargets();
    RCLCPP_INFO(get_logger(), "Available named targets:");
    for (const auto& target : named_targets) {
        RCLCPP_INFO(get_logger(), "  - %s", target.c_str());
    }
    
    move_group_interface_->setNamedTarget(config_name);
    moveit::planning_interface::MoveGroupInterface::Plan config_plan;
    bool success = static_cast<bool>(move_group_interface_->plan(config_plan));
    
    if (success) {
        RCLCPP_INFO(get_logger(), "Configuration plan found! Executing...");
        move_group_interface_->execute(config_plan);
        RCLCPP_INFO(get_logger(), "Successfully moved to configuration: %s", config_name.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
        return true;
    }
    
    RCLCPP_ERROR(get_logger(), "Failed to plan move to configuration: %s", config_name.c_str());
    return false;
}

geometry_msgs::msg::Pose TagPicker::calculateBaseAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset)
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
    
    // Direction toward base (normalized) - with safety check for division by zero
    double distance_to_base = sqrt(tag_x * tag_x + tag_y * tag_y);
    tf2::Vector3 base_direction;
    
    if (distance_to_base < MovementConstants::MIN_DISTANCE_TO_BASE) {  // Very close to base
        RCLCPP_WARN(get_logger(), "TF frame very close to robot base (distance: %.6f), using default Y-axis alignment", distance_to_base);
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

std::vector<geometry_msgs::msg::Pose> TagPicker::calculateBaseAlignedPoses(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset)
{
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
        
        RCLCPP_DEBUG(get_logger(), "Generated pose for X-axis rotation %d°: position(%.3f, %.3f, %.3f)", 
                    angle_deg, pose.position.x, pose.position.y, pose.position.z);
    }
    
    RCLCPP_INFO(get_logger(), "Generated %zu poses with X-axis rotations: 0°, -15°, -30°, -45°", poses.size());
    
    return poses;
}

bool TagPicker::updateStoredTagIfVisible(int tag_id)
{
    RCLCPP_INFO(get_logger(), "Checking if tag ID %d is visible for pose update...", tag_id);
    
    std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
    
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

bool TagPicker::moveToReacquireTagPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id)
{
    // Use look-at pose for approach position to orient TCP toward tag center
    auto tag_pose = calculateBaseAlignedPose(tag_transform, MovementConstants::CAM_HEIGHT + MovementConstants::APPROACH_HEIGHT);

    RCLCPP_INFO(get_logger(), "Moving to tag position (%.1fcm above) with look-at orientation: x=%.3f, y=%.3f, z=%.3f",
               MovementConstants::CAM_HEIGHT * 100, tag_pose.position.x,
               tag_pose.position.y, tag_pose.position.z);
               
    move_group_interface_->clearPoseTargets();
    move_group_interface_->setPoseTarget(tag_pose, "jetcocam");

    // Plan and execute approach motion
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    bool success = static_cast<bool>(move_group_interface_->plan(approach_plan));
    
    if (success) {
        RCLCPP_INFO(get_logger(), "Approach plan found! Executing...");
        move_group_interface_->execute(approach_plan);
        RCLCPP_INFO(get_logger(), "Approach motion completed!");
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
        
        // Try to update the stored tag pose if the tag is visible from the new position
        if (tag_id >= 0) {
            updateStoredTagIfVisible(tag_id);
        }
        
        return true;
    }
    
    return false;
}

bool TagPicker::executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description)
{
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface_->computeCartesianPath(waypoints, MovementConstants::EEF_STEP, trajectory);

    if (fraction > MovementConstants::MIN_PATH_FRACTION) {
        RCLCPP_INFO(get_logger(), "%s (path fraction: %.2f)", description.c_str(), fraction);
        move_group_interface_->execute(trajectory);
        return true;
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to plan %s! Cartesian path fraction: %.2f", description.c_str(), fraction);
        return false;
    }
}

bool TagPicker::executeStabilizedMovement(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description)
{
    if (!executeCartesianPath(waypoints, description)) {
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "Movement completed: %s", description.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::STABILIZE_DELAY_MS));
    return true;
}

bool TagPicker::executeLiftMovement(double lift_height)
{
    geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose("TCP").pose;
    current_pose.position.z += lift_height;
    
    std::vector<geometry_msgs::msg::Pose> lift_waypoints{current_pose};
    return executeStabilizedMovement(lift_waypoints, "lifting movement");
}

void TagPicker::controlGripper(int close_value)
{
    auto gripper_msg = std_msgs::msg::Int32();
    gripper_msg.data = close_value;
    gripper_pub_->publish(gripper_msg);
    
    if (close_value == GripperPositions::FULLY_OPEN) {
        RCLCPP_INFO(get_logger(), "Opening gripper fully...");
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    } else if (close_value == GripperPositions::FULLY_CLOSED) {
        RCLCPP_INFO(get_logger(), "Closing gripper fully...");
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::GRIPPER_CLOSE_DELAY_MS));
    } else {
        RCLCPP_INFO(get_logger(), "Setting gripper position to %d", close_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    }
}

bool TagPicker::findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform)
{
    std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
    
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

bool TagPicker::executePick(int tag_id)
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
    
    // Get the potentially updated tag transform after approach
    geometry_msgs::msg::TransformStamped updated_tag_transform;
    if (getStoredTagTransform(tag_id, updated_tag_transform)) {
        tag_transform = updated_tag_transform;
        RCLCPP_INFO(get_logger(), "Using updated tag transform for final approach");
    }

     // Get current EEF orientation
    geometry_msgs::msg::Pose current_ee_pose = move_group_interface_->getCurrentPose("TCP").pose;
    auto current_ee_orientation = current_ee_pose.orientation;
    
    // Calculate multiple final target poses with different X-axis rotations
    auto final_target_poses = calculateBaseAlignedPoses(tag_transform, MovementConstants::PICK_HEIGHT);
    
    // Try each pose for final approach until one succeeds
    bool final_approach_success = false;
    geometry_msgs::msg::Pose successful_pose;
    const auto& angles = RotationAngles::APPROACH_ANGLES;
    
    for (size_t i = 0; i < final_target_poses.size(); ++i) {
        RCLCPP_INFO(get_logger(), "Attempting final approach %zu/%zu (X-rotation: %d°)", 
                   i + 1, final_target_poses.size(), angles[i]);
        final_target_poses[i].orientation = current_ee_orientation;  // Maintain current EEF orientation
        // Move to final position using Cartesian path
        std::vector<geometry_msgs::msg::Pose> approach_waypoints{final_target_poses[i]};
        if (executeCartesianPath(approach_waypoints, "final approach to tag")) {
            final_approach_success = true;
            successful_pose = final_target_poses[i];  // Store the successful pose
            RCLCPP_INFO(get_logger(), "Final approach successful with %d° X-rotation!", angles[i]);
            break;
        } else {
            RCLCPP_WARN(get_logger(), "Final approach failed with %d° X-rotation, trying next angle...", angles[i]);
        }
    }
    
    if (!final_approach_success) {
        RCLCPP_ERROR(get_logger(), "All final approach attempts failed!");
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "Final aligned motion completed!");
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::STABILIZE_DELAY_MS));
    
    // Close gripper
    closeGripperToPicking();

    // Lift object - use the successful pose for lifting
    if (!executeLiftMovement(MovementConstants::LIFT_HEIGHT)) {
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "Pick operation completed for tag ID: %d", tag_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    
    return true;
}

bool TagPicker::executePlace(int target_tag_id, int source_tag_id)
{
    RCLCPP_INFO(get_logger(), "Starting place operation: placing source tag %d at target tag %d position", source_tag_id, target_tag_id);
    
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
    
    // Get the potentially updated target tag transform after approach
    geometry_msgs::msg::TransformStamped updated_target_tag_transform;
    if (getStoredTagTransform(target_tag_id, updated_target_tag_transform)) {
        target_tag_transform = updated_target_tag_transform;
        RCLCPP_INFO(get_logger(), "Using updated target tag transform for placement");
    }
    
    // Get current EEF orientation
    geometry_msgs::msg::Pose current_ee_pose = move_group_interface_->getCurrentPose("TCP").pose;
    auto current_ee_orientation = current_ee_pose.orientation;

    // Calculate placement pose using (potentially updated) stored transform
    auto place_pose = calculateBaseAlignedPose(target_tag_transform, MovementConstants::PLACE_HEIGHT);
    place_pose.orientation = current_ee_orientation;  // Maintain current EEF orientation
    
    // Move down to placement position
    std::vector<geometry_msgs::msg::Pose> place_waypoints{place_pose};
    if (!executeStabilizedMovement(place_waypoints, "moving to placement position")) {
        return false;
    }
    
    // Open gripper to release object
    openGripperToHoldingPosition();

    // Move up to lift position
    if (!executeLiftMovement(MovementConstants::APPROACH_HEIGHT)) {
        return false;
    }
    
    // Update stored tag pose after successful placement
    RCLCPP_INFO(get_logger(), "Updating stored pose for placed source tag ID %d (now at target position)...", source_tag_id);
    updateStoredTagIfVisible(source_tag_id);
    
    RCLCPP_INFO(get_logger(), "Place operation completed: source tag %d placed at target tag %d position", source_tag_id, target_tag_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    
    return true;
}

bool TagPicker::executePlace(const std::string& target_tf_name, int source_tag_id)
{
    RCLCPP_INFO(get_logger(), "Starting place operation: placing source tag %d at TF frame: %s", source_tag_id, target_tf_name.c_str());
    
    // Get transform for the specified TF frame
    geometry_msgs::msg::TransformStamped target_transform;
    try {
        target_transform = tf_buffer_->lookupTransform(
            "base_link",  // target frame
            target_tf_name,  // source frame  
            tf2::TimePointZero,  // get latest available
            std::chrono::seconds(1));
        
        RCLCPP_INFO(get_logger(), "Found TF frame: %s at position: x=%.3f, y=%.3f, z=%.3f", 
                   target_tf_name.c_str(),
                   target_transform.transform.translation.x,
                   target_transform.transform.translation.y,
                   target_transform.transform.translation.z);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Failed to find TF frame %s: %s", target_tf_name.c_str(), ex.what());
        return false;
    }

    // First, move to approach position (APPROACH_HEIGHT above the target frame)
    auto approach_pose = calculateBaseAlignedPose(target_transform, MovementConstants::APPROACH_HEIGHT);

    RCLCPP_INFO(get_logger(), "Moving to approach position (%.1fcm above) for TF frame: x=%.3f, y=%.3f, z=%.3f",
               MovementConstants::APPROACH_HEIGHT * 100, approach_pose.position.x,
               approach_pose.position.y, approach_pose.position.z);

    move_group_interface_->clearPoseTargets();
    move_group_interface_->setPoseTarget(approach_pose, "TCP");

    // Plan and execute approach motion
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    bool approach_success = static_cast<bool>(move_group_interface_->plan(approach_plan));
    
    if (!approach_success) {
        RCLCPP_ERROR(get_logger(), "Failed to plan move to approach position for TF frame");
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "Approach plan found! Executing...");
    move_group_interface_->execute(approach_plan);
    RCLCPP_INFO(get_logger(), "Successfully moved to approach position!");
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));

    // Then, move down to final placement position using Cartesian path
    auto place_pose = calculateBaseAlignedPose(target_transform, MovementConstants::PLACE_HEIGHT);
    
    RCLCPP_INFO(get_logger(), "Moving to final placement position using Cartesian path: x=%.3f, y=%.3f, z=%.3f",
               place_pose.position.x, place_pose.position.y, place_pose.position.z);
    
    // Move to placement position using Cartesian path
    std::vector<geometry_msgs::msg::Pose> place_waypoints{place_pose};
    if (!executeStabilizedMovement(place_waypoints, "moving to TF frame placement position")) {
        return false;
    }
    
    // Open gripper to release object
    openGripperToHoldingPosition();
    
    // Move up to lift position after placing
    if (!executeLiftMovement(MovementConstants::APPROACH_HEIGHT)) {
        return false;
    }

    // Update stored tag pose after successful placement
    RCLCPP_INFO(get_logger(), "Updating stored pose for placed source tag ID %d (now at TF frame position)...", source_tag_id);
    updateStoredTagIfVisible(source_tag_id);

    RCLCPP_INFO(get_logger(), "Place operation completed: source tag %d placed at TF frame: %s", source_tag_id, target_tf_name.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(TimingConstants::OPERATION_DELAY_MS));
    
    return true;
}

bool TagPicker::handleHomeCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing HOME command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_home";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    bool success = moveToConfiguration("ready_to_see");
    
    if (!success) {
        RCLCPP_ERROR(get_logger(), "Failed to move to home position");
    } else {
        RCLCPP_INFO(get_logger(), "HOME command completed successfully");
    }
    
    return success;
}

bool TagPicker::handleScanCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing SCAN command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "searching";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    bool success = collectDetectedTagsAndAcquireTransforms(TimingConstants::TAG_COLLECTION_TIME);
    
    if (!success) {
        RCLCPP_ERROR(get_logger(), "SCAN command failed - no tags detected");
    } else {
        printDetectedTags();
        RCLCPP_INFO(get_logger(), "SCAN command completed successfully");
    }
    
    return success;
}

bool TagPicker::handleScanFrontCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing SCAN_FRONT command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_scan_position";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    // Move to scan_front configuration
    if (!moveToConfiguration("scan_front")) {
        RCLCPP_ERROR(get_logger(), "Failed to move to scan_front configuration");
        return false;
    }
    
    feedback->current_phase = "searching";
    goal_handle->publish_feedback(feedback);
    
    bool success = collectDetectedTagsAndAcquireTransforms(TimingConstants::TAG_COLLECTION_TIME);
    
    if (!success) {
        RCLCPP_ERROR(get_logger(), "SCAN_FRONT command failed - no tags detected");
    } else {
        printDetectedTags();
        RCLCPP_INFO(get_logger(), "SCAN_FRONT command completed successfully");
    }
    
    return success;
}

bool TagPicker::handleScanLeftCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing SCAN_LEFT command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_scan_position";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    // Move to scan_left configuration
    if (!moveToConfiguration("scan_left")) {
        RCLCPP_ERROR(get_logger(), "Failed to move to scan_left configuration");
        return false;
    }
    
    feedback->current_phase = "searching";
    goal_handle->publish_feedback(feedback);
    
    bool success = collectDetectedTagsAndAcquireTransforms(TimingConstants::TAG_COLLECTION_TIME);
    
    if (!success) {
        RCLCPP_ERROR(get_logger(), "SCAN_LEFT command failed - no tags detected");
    } else {
        printDetectedTags();
        RCLCPP_INFO(get_logger(), "SCAN_LEFT command completed successfully");
    }
    
    return success;
}

bool TagPicker::handleScanRightCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing SCAN_RIGHT command");
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    feedback->current_phase = "moving_to_scan_position";
    feedback->current_tag_id = -1;
    goal_handle->publish_feedback(feedback);
    
    // Move to scan_right configuration
    if (!moveToConfiguration("scan_right")) {
        RCLCPP_ERROR(get_logger(), "Failed to move to scan_right configuration");
        return false;
    }
    
    feedback->current_phase = "searching";
    goal_handle->publish_feedback(feedback);
    
    bool success = collectDetectedTagsAndAcquireTransforms(TimingConstants::TAG_COLLECTION_TIME);
    
    if (!success) {
        RCLCPP_ERROR(get_logger(), "SCAN_RIGHT command failed - no tags detected");
    } else {
        printDetectedTags();
        RCLCPP_INFO(get_logger(), "SCAN_RIGHT command completed successfully");
    }
    
    return success;
}

bool TagPicker::handlePickAndPlaceCommand(const std::shared_ptr<GoalHandlePickerAction> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "Executing PICK_AND_PLACE command: source_id=%d, target_id=%d, target_tf_name='%s'", 
               goal->source_tag_id, goal->target_tag_id, goal->target_tf_name.c_str());
    
    auto feedback = std::make_shared<PickerAction::Feedback>();
    
    // Execute pick operation
    feedback->current_phase = "approaching_source";
    feedback->current_tag_id = goal->source_tag_id;
    goal_handle->publish_feedback(feedback);
    
    feedback->current_phase = "picking";
    goal_handle->publish_feedback(feedback);
    
    if (!executePick(goal->source_tag_id)) {
        RCLCPP_ERROR(get_logger(), "Pick operation failed for tag ID %d", goal->source_tag_id);
        return false;
    }
    
    // Execute place operation - check if using TF name or tag ID
    feedback->current_phase = "moving_to_target";
    if (!goal->target_tf_name.empty()) {
        // Use TF frame name
        feedback->current_tag_id = -1;  // No tag ID when using TF name
        goal_handle->publish_feedback(feedback);
        
        feedback->current_phase = "placing";
        goal_handle->publish_feedback(feedback);
        
        if (!executePlace(goal->target_tf_name, goal->source_tag_id)) {
            RCLCPP_ERROR(get_logger(), "Place operation failed for TF frame %s", goal->target_tf_name.c_str());
            return false;
        }
    } else {
        // Use tag ID
        feedback->current_tag_id = goal->target_tag_id;
        goal_handle->publish_feedback(feedback);
        
        feedback->current_phase = "placing";
        goal_handle->publish_feedback(feedback);
        
        if (!executePlace(goal->target_tag_id, goal->source_tag_id)) {
            RCLCPP_ERROR(get_logger(), "Place operation failed for tag ID %d", goal->target_tag_id);
            return false;
        }
    }
    
    RCLCPP_INFO(get_logger(), "PICK_AND_PLACE command completed successfully");
    return true;
}

// Main function
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
