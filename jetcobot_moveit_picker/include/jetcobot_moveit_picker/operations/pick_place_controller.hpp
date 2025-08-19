#ifndef PICK_PLACE_CONTROLLER_HPP
#define PICK_PLACE_CONTROLLER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Forward declarations
namespace jetcobot_picker {
    class RobotController;
    class TagManager;
    class CollisionManager;
}

namespace jetcobot_picker {

/**
 * @brief High-level pick and place operation controller
 */
class PickPlaceController 
{
public:
    /**
     * @brief Constructor
     * @param node ROS node pointer for logging
     * @param robot_controller Robot movement controller
     * @param tag_manager Tag detection and management
     * @param collision_manager Collision object management
     */
    PickPlaceController(
        rclcpp::Node* node,
        std::shared_ptr<RobotController> robot_controller,
        std::shared_ptr<TagManager> tag_manager,
        std::shared_ptr<CollisionManager> collision_manager
    );

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

private:
    rclcpp::Node* node_;
    std::shared_ptr<RobotController> robot_controller_;
    std::shared_ptr<TagManager> tag_manager_;
    std::shared_ptr<CollisionManager> collision_manager_;
};

} // namespace jetcobot_picker

#endif // PICK_PLACE_CONTROLLER_HPP
