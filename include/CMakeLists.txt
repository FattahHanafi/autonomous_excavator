cmake_minimum_required(VERSION 3.8)
project(MarchinCubesClass)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add source files
file(GLOB_RECURSE SOURCE_FILES *.cpp)
	
# Add header files
file(GLOB_RECURSE HEADER_FILES *.h)

# Define the executable
add_library(MarchinCubesClass STATIC ${HEADER_FILES} ${SOURCE_FILES})
ament_target_dependencies(MarchinCubesClass geometry_msgs sensor_msgs std_msgs visualization_msgs) 
