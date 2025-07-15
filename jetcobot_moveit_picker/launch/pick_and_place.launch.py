from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("jetcobot")
        .robot_description(file_path="config/firefighter.urdf.xacro")
        .robot_description_semantic(file_path="config/firefighter.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="jetcobot_moveit_picker",
                executable="tag_picker",
                name="tag_picker",
                output="screen",
                parameters=[moveit_config.to_dict()],
            )
        ]
    )