cmake_minimum_required(VERSION 2.8.3)
project(transform_sensors)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	geometry_msgs
	sensor_msgs
    tf2
    tf2_ros
    tf2_msgs
    tf2_sensor_msgs
    laser_geometry
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(transform_sensors src/main.cpp src/transform_sensors.cpp)
target_link_libraries(transform_sensors ${catkin_LIBRARIES})

