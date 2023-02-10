cmake_minimum_required(VERSION 3.20)
project(MarchinCubesClass)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
 set(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
