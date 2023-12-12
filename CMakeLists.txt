cmake_minimum_required(VERSION 3.8)
project(uclv_yaskawa_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uclv_moveit_planner_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uclv_utilities REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(yaskawa_wsg_pub src/yaskawa_wsg_pub.cpp)
ament_target_dependencies(yaskawa_wsg_pub rclcpp sensor_msgs)

add_executable(twist_ref_pub src/twist_ref_pub.cpp)
ament_target_dependencies(twist_ref_pub rclcpp sensor_msgs geometry_msgs uclv_moveit_planner_ros2 std_msgs uclv_utilities tf2_ros moveit_ros_planning_interface moveit_msgs) 

add_executable(angle_control_node src/angle_control_node.cpp)
ament_target_dependencies(angle_control_node rclcpp geometry_msgs std_msgs uclv_utilities tf2_ros tf2) 

<<<<<<< HEAD
=======
add_executable(force_control_node src/force_control_node.cpp)
ament_target_dependencies(force_control_node rclcpp geometry_msgs std_msgs uclv_utilities tf2_ros tf2) 

>>>>>>> 14fa28f5b0662a0ff8006b37cbcfd76033c1e09c
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch config description rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  yaskawa_wsg_pub
  twist_ref_pub
  angle_control_node
<<<<<<< HEAD
=======
  force_control_node
>>>>>>> 14fa28f5b0662a0ff8006b37cbcfd76033c1e09c
  DESTINATION lib/${PROJECT_NAME})

ament_package()
