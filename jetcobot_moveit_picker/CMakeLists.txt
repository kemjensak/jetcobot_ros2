cmake_minimum_required(VERSION 3.8)
project(jetcobot_moveit_picker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jetcobot_interfaces REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)

# Optional packages - check if available
find_package(rviz_visual_tools QUIET)
find_package(moveit_visual_tools QUIET)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  std_msgs
  jetcobot_interfaces
  apriltag_msgs
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
)

# Add optional dependencies if found
if(rviz_visual_tools_FOUND)
  list(APPEND THIS_PACKAGE_INCLUDE_DEPENDS rviz_visual_tools)
endif()

if(moveit_visual_tools_FOUND)
  list(APPEND THIS_PACKAGE_INCLUDE_DEPENDS moveit_visual_tools)
endif()

add_executable(tag_picker
    src/tag_picker.cpp)
target_include_directories(tag_picker
    PUBLIC include)
ament_target_dependencies(tag_picker
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables as ROS 2 nodes
install(PROGRAMS
      scripts/moveit_mover.py
      # Add more Python scripts here as needed
      # scripts/another_script.py
      DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tag_picker
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/task_status.py
  scripts/picker_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(jetcobot_interfaces)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
