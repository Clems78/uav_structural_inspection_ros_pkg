cmake_minimum_required(VERSION 3.8)
project(irp_ros_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)


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

# Build 
add_executable(wp_nav src/wp_nav.cpp)
ament_target_dependencies(wp_nav rclcpp px4_msgs px4_ros_com std_srvs)

add_executable(wp_camera_service_server src/wp_camera_service_server.cpp)
ament_target_dependencies(wp_camera_service_server rclcpp std_msgs sensor_msgs cv_bridge OpenCV std_srvs px4_msgs geometry_msgs)

#add_executable(waypoint_navigator src/waypoint_navigator.cpp)
#ament_target_dependencies(waypoint_navigator rclcpp px4_msgs px4_ros_com std_srvs)

# Install
install(TARGETS
  wp_nav
  wp_camera_service_server
  #waypoint_navigator
  DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
