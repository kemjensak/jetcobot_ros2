from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import conditions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Get the URDF file path
    package_dir = get_package_share_directory('jetcobot_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'jetcobot_display.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Get GUI parameter
    gui = LaunchConfiguration('gui').perform(context)

    nodes_to_launch = []

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    nodes_to_launch.append(robot_state_publisher_node)

    # Joint State Publisher (with or without GUI)
    if gui == 'false':
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        )
        nodes_to_launch.append(joint_state_publisher_node)
    else:
        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        )
        nodes_to_launch.append(joint_state_publisher_gui_node)

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    nodes_to_launch.append(rviz_node)

    return nodes_to_launch

def generate_launch_description():
    ld = LaunchDescription()

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='true', 
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    ld.add_action(gui_arg)

    # Use OpaqueFunction to handle context-dependent operations
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
