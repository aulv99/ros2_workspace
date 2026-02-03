cmake_minimum_required(VERSION 3.8)
project(amr_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

add_executable(wall_stop src/wall_stop.cpp)
ament_target_dependencies(wall_stop rclcpp sensor_msgs geometry_msgs)

add_executable(go_to_goal src/go_to_goal.cpp)
target_compile_features(go_to_goal PUBLIC cxx_std_17)
ament_target_dependencies(go_to_goal 
    rclcpp 
    sensor_msgs 
    geometry_msgs 
    nav_msgs 
    tf2 
    tf2_geometry_msgs
)

add_executable(waypoint_patrol src/waypoint_patrol.cpp)
target_compile_features(waypoint_patrol PUBLIC cxx_std_17)
ament_target_dependencies(waypoint_patrol 
    rclcpp 
    sensor_msgs 
    geometry_msgs 
    nav_msgs 
    tf2 
    tf2_geometry_msgs
)

install(TARGETS
  wall_stop
  go_to_goal
  waypoint_patrol
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
