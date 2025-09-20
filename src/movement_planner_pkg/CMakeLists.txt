cmake_minimum_required(VERSION 3.10)
project(movement_planner_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(path_planning_node src/path_planning_node.cpp)
add_executable(RB_builder_node src/RB_builder_node.cpp)
add_executable(mtc_node src/mtc_node.cpp)

ament_target_dependencies(path_planning_node 
                          rclcpp 
                          tf2 
                          tf2_ros 
                          tf2_geometry_msgs 
                          geometry_msgs 
                          moveit_ros_planning_interface)
ament_target_dependencies(RB_builder_node 
                          rclcpp 
                          tf2 
                          tf2_ros 
                          tf2_geometry_msgs 
                          geometry_msgs 
                          moveit_ros_planning_interface 
                          moveit_msgs 
                          shape_msgs 
                          foxglove_msgs)
ament_target_dependencies(mtc_node 
                          rclcpp 
                          tf2 
                          tf2_ros 
                          tf2_geometry_msgs 
                          geometry_msgs 
                          moveit_ros_planning_interface 
                          moveit_task_constructor_core)

target_include_directories(path_planning_node PUBLIC include)
target_include_directories(RB_builder_node PUBLIC include)
target_include_directories(mtc_node PUBLIC include)

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

install(
  TARGETS path_planning_node RB_builder_node mtc_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
