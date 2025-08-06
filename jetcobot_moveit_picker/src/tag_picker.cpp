#include "jetcobot_moveit_picker/tag_picker.hpp"

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
    
    command_sub_ = create_subscription<std_msgs::msg::String>(
        "/picker/command", 10,
        std::bind(&TagPicker::commandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "TagPicker initialized successfully");
}

bool TagPicker::execute()
{
    // Initialize MoveGroupInterface after the object is fully constructed
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    move_group_interface_->setMaxAccelerationScalingFactor(0.6);
    move_group_interface_->setMaxVelocityScalingFactor(0.9);
    move_group_interface_->setPlanningTime(15.0);  // Set planning time to 15 seconds
    move_group_interface_->setNumPlanningAttempts(100);
    
    // Move to home position first
    if (!moveToHome()) {
        RCLCPP_WARN(get_logger(), "Failed to move to home position, continuing anyway...");
    }

    controlGripper(100);  // Open gripper fully to start fresh

    // Use detection-based tag discovery
    if (!collectDetectedTagsAndAcquireTransforms(1.0)) {
        RCLCPP_ERROR(get_logger(), "No AprilTags detected! Make sure AprilTag detection is running on /detections topic.");
        return false;
    }
    
    // Show all detected tags
    printDetectedTags();
    
    // Example usage of new pick and place functions
    // Pick from tag ID 9 and place at tag ID 7
    int pick_tag_id = 9;
    if (!executePick(pick_tag_id)) {
        RCLCPP_ERROR(get_logger(), "Pick operation failed for tag ID %d", pick_tag_id);
        return false;
    }
    
    // Place operation
    int place_tag_id = 7;
    if (!executePlace(place_tag_id)) {
        RCLCPP_ERROR(get_logger(), "Place operation failed for tag ID %d", place_tag_id);
        return false;
    }

    // Return to home position
    if (!moveToHome()) {
        RCLCPP_WARN(get_logger(), "Failed to return to home position");
    }
    
    // Re-scan for tags before second operation
    if (!collectDetectedTagsAndAcquireTransforms(1.0)) {
        RCLCPP_ERROR(get_logger(), "No AprilTags detected for second operation! Make sure AprilTag detection is running on /detections topic.");
        return false;
    }
    
    // Show all detected tags for second operation
    printDetectedTags();

    pick_tag_id = 6;
    if (!executePick(pick_tag_id)) {
        RCLCPP_ERROR(get_logger(), "Pick operation failed for tag ID %d", pick_tag_id);
        return false;
    }

    place_tag_id = 9;
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

void TagPicker::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received command: %s", msg->data.c_str());
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
    stored_tag_transforms_.clear();
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
        
        // Spin to process callbacks
        rclcpp::spin_some(shared_from_this());
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

bool TagPicker::moveToHome()
{
    RCLCPP_INFO(get_logger(), "Moving to home position...");
    
    // Get available named targets for debugging
    std::vector<std::string> named_targets = move_group_interface_->getNamedTargets();
    RCLCPP_INFO(get_logger(), "Available named targets:");
    for (const auto& target : named_targets) {
        RCLCPP_INFO(get_logger(), "  - %s", target.c_str());
    }
    
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

bool TagPicker::executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description)
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

void TagPicker::controlGripper(int close_value)
{
    auto gripper_msg = std_msgs::msg::Int32();
    gripper_msg.data = close_value;
    gripper_pub_->publish(gripper_msg);
    
    if (close_value == 100) {
        RCLCPP_INFO(get_logger(), "Opening gripper fully...");
        std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
    } else if (close_value == 0) {
        RCLCPP_INFO(get_logger(), "Closing gripper fully...");
        std::this_thread::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_DELAY_MS));
    } else {
        RCLCPP_INFO(get_logger(), "Setting gripper position to %d", close_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
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
    controlGripper(0); // Close gripper fully

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

bool TagPicker::executePlace(int target_tag_id)
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
    controlGripper(100); // Open gripper fully
    
    RCLCPP_INFO(get_logger(), "Place operation completed at tag ID: %d", target_tag_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
    
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
