cmake_minimum_required(VERSION 3.8)
project(autonomous_excavator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(GSL REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(CGAL REQUIRED)

add_executable(joint_publisher src/joint_publisher.cpp)
ament_target_dependencies(joint_publisher rclcpp sensor_msgs)

add_executable(stroke_publisher src/stroke_publisher.cpp)
ament_target_dependencies(stroke_publisher rclcpp sensor_msgs)

add_executable(home_stroke_publisher src/home_stroke_publisher.cpp)
ament_target_dependencies(home_stroke_publisher rclcpp sensor_msgs)

add_executable(feedback_logger src/feedback_logger.cpp)
ament_target_dependencies(feedback_logger rclcpp sensor_msgs std_msgs)

add_executable(excavator_tf2_broadcaster src/excavator_tf2_broadcaster.cpp)
ament_target_dependencies(excavator_tf2_broadcaster rclcpp geometry_msgs sensor_msgs tf2 tf2_ros)

add_executable(blade_node src/blade_node.cpp)
ament_target_dependencies(blade_node rclcpp geometry_msgs sensor_msgs)

add_executable(surface_reconstructor src/surface_reconstructor.cpp)
ament_target_dependencies(surface_reconstructor rclcpp std_msgs sensor_msgs visualization_msgs)

add_executable(merged src/merged.cpp)
ament_target_dependencies(merged rclcpp sensor_msgs tf2_msgs tf2_ros nav_msgs realsense2 cv_bridge GSL cv_bridge diagnostic_updater CGAL)
target_include_directories(merged PRIVATE ${realsense2_INCLUDE_DIR} /opt/ros/humble/include/realsense2_camera_msgs/)
target_link_libraries(merged ${realsense2_LIBRARY})

add_executable(serial_interface src/serial_interface.cpp)
ament_target_dependencies(serial_interface rclcpp sensor_msgs std_msgs)

add_executable(marching_cubes_node src/marching_cubes_node.cpp)
ament_target_dependencies(marching_cubes_node rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_msgs tf2_geometry_msgs tf2_sensor_msgs)
target_include_directories(marching_cubes_node PRIVATE include/MarchingCubes.h)
target_link_libraries(marching_cubes_node MarchinCubesClass)

install(TARGETS
  joint_publisher
  stroke_publisher
  home_stroke_publisher
  excavator_tf2_broadcaster
  blade_node
  surface_reconstructor
  merged
  serial_interface
  marching_cubes_node
  feedback_logger
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
