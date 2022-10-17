cmake_minimum_required(VERSION 3.8)
project(autonomous_excavator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GSL REQUIRED)

add_executable(excavator_tf2_broadcaster src/excavator_tf2_broadcaster.cpp)
ament_target_dependencies(excavator_tf2_broadcaster rclcpp geometry_msgs sensor_msgs tf2 tf2_ros)

add_executable(camera_reader src/camera_reader.cpp)
ament_target_dependencies(camera_reader rclcpp sensor_msgs realsense2 cv_bridge GSL)
include_directories(include ${realsense_INCLUDE_DIR})
target_include_directories(camera_reader PRIVATE ${realsense_INCLUDE_DIR})
target_link_libraries(camera_reader ${realsense2_LIBRARY})

add_executable(roi_node src/roi_node.cpp)
ament_target_dependencies(roi_node rclcpp sensor_msgs std_msgs cv_bridge)


install(TARGETS
  excavator_tf2_broadcaster
  camera_reader
  roi_node
  #  surface_reconstructor
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
