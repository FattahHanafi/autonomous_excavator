cmake_minimum_required(VERSION 3.8)
project(autonomous_excavator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# se(CMAKE_BUILD_TYPE Debug) set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autonomous_excavator_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(GSL REQUIRED)
find_package(CGAL REQUIRED)

include_directories(include)

add_library(motion_server SHARED src/motion_action_server.cpp)
ament_target_dependencies(motion_server rclcpp rclcpp_action rclcpp_components
                          autonomous_excavator_interfaces)

# add_library(camera_server SHARED src/camera_action_server.cpp)
# ament_target_dependencies( camera_server rclcpp rclcpp_action
# rclcpp_components autonomous_excavator_interfaces std_msgs sensor_msgs)

target_include_directories(
  motion_server PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)

# rclcpp_components_register_node( motion_server PLUGIN
# "autonomous_excavator::MotionActionServer" EXECUTABLE motion_action_server)

# rclcpp_components_register_node( camera_server PLUGIN
# "autonomous_excavator::CameraActionServer" EXECUTABLE camera_action_server)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(
  camera_node
  rclcpp
  rclcpp_action
  rclcpp_components
  autonomous_excavator_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  GSL
  CGAL)

add_executable(joint_publisher src/joint_publisher.cpp)
ament_target_dependencies(joint_publisher rclcpp sensor_msgs)

add_executable(stroke_publisher src/stroke_publisher.cpp)
ament_target_dependencies(stroke_publisher rclcpp sensor_msgs)

add_executable(excavator_tf2_broadcaster src/excavator_tf2_broadcaster.cpp)
ament_target_dependencies(excavator_tf2_broadcaster rclcpp geometry_msgs
                          sensor_msgs tf2 tf2_ros)

# install(TARGETS motion_server camera_server)

install(TARGETS camera_node joint_publisher stroke_publisher
                excavator_tf2_broadcaster DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY launch urdf config DESTINATION share/${PROJECT_NAME}/)

# Install Python executables
install(PROGRAMS scripts/udp_rx_node.py scripts/udp_tx_node.py
        DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
