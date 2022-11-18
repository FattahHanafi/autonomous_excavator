cmake_minimum_required(VERSION 3.8)
project(autonomous_excavator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost REQUIRED)
find_package(diagnostic_updater REQUIRED)
# find_package(MarchingCubesClass REQUIRED)

# add_library(Vec3 include/Vec3.cpp include/Vec3.h)

# add_library(Blade include/Blade.cpp include/Blade.h)
# ament_target_dependencies(Blade rclcpp std_msgs geometry_msgs sensor_msgs)
# target_link_libraries(Blade Vec3)
# target_include_directories(Blade PRIVATE include/)

# add_library(MarchingCubes include/MarchingCubes.cpp include/MarchingCubes.h)
# ament_target_dependencies(MarchingCubes rclcpp std_msgs geometry_msgs sensor_msgs)
# target_link_libraries(MarchingCubes Blade Vec3)
# target_include_directories(MarchingCubes PRIVATE include/)

# add_library(MyLibExample include/MarchingCubes.cpp include/Vec3.cpp include/Blade.cpp)
# ament_target_dependencies(MyLibExample geometry_msgs std_msgs sensor_msgs)

add_executable(joint_publisher src/joint_publisher.cpp)
ament_target_dependencies(joint_publisher rclcpp sensor_msgs)

add_executable(stroke_publisher src/stroke_publisher.cpp)
ament_target_dependencies(stroke_publisher rclcpp sensor_msgs)

add_executable(feedback_logger src/feedback_logger.cpp)
ament_target_dependencies(feedback_logger rclcpp sensor_msgs std_msgs)

add_executable(excavator_tf2_broadcaster src/excavator_tf2_broadcaster.cpp)
ament_target_dependencies(excavator_tf2_broadcaster rclcpp geometry_msgs sensor_msgs tf2 tf2_ros)

add_executable(blade_node src/blade_node.cpp)
ament_target_dependencies(blade_node rclcpp geometry_msgs sensor_msgs)

add_executable(camera_reader src/camera_reader.cpp)
ament_target_dependencies(camera_reader rclcpp sensor_msgs realsense2 cv_bridge GSL)
target_include_directories(camera_reader PRIVATE ${realsense2_INCLUDE_DIR})
target_link_libraries(camera_reader ${realsense2_LIBRARY})

add_executable(roi_node src/roi_node.cpp)
ament_target_dependencies(roi_node rclcpp sensor_msgs std_msgs cv_bridge)

add_executable(surface_reconstructor src/surface_reconstructor.cpp)
ament_target_dependencies(surface_reconstructor rclcpp std_msgs sensor_msgs GSL cv_bridge)

add_executable(depth_to_point_cloud src/depth_image_to_point_cloud.cpp)
ament_target_dependencies(depth_to_point_cloud rclcpp std_msgs sensor_msgs realsense2)
target_include_directories(depth_to_point_cloud PRIVATE ${realsense2_INCLUDE_DIR})
target_link_libraries(depth_to_point_cloud ${realsense2_LIBRARY})

add_executable(merged src/merged.cpp)
ament_target_dependencies(merged rclcpp sensor_msgs tf2_msgs tf2_ros nav_msgs realsense2 cv_bridge GSL cv_bridge diagnostic_updater)
target_include_directories(merged PRIVATE ${realsense2_INCLUDE_DIR} /opt/ros/humble/include/realsense2_camera_msgs/)
target_link_libraries(merged ${realsense2_LIBRARY})

add_executable(serial_interface src/serial_interface.cpp)
ament_target_dependencies(serial_interface rclcpp sensor_msgs std_msgs)

add_executable(marching_cubes_node src/marching_cubes_node.cpp)
ament_target_dependencies(marching_cubes_node rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_msgs tf2_geometry_msgs tf2_sensor_msgs pcl_ros)
target_include_directories(marching_cubes_node PRIVATE include/MarchingCubes.h)
target_link_libraries(marching_cubes_node MarchinCubesClass)

install(TARGETS
  joint_publisher
  stroke_publisher
  excavator_tf2_broadcaster
  blade_node
  camera_reader
  roi_node
  surface_reconstructor
  depth_to_point_cloud
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
