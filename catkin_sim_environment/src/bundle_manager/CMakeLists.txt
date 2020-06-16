cmake_minimum_required(VERSION 2.8.3)
project(bundle_manager)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  gencpp
  geometry_msgs
  message_generation
)

## Define custom message files
add_message_files(
  FILES
  BUNDLE_CONTENT.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES

  BUNDLE_REQUEST.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs message_runtime
  LIBRARIES bundle_manager
)

# include_directories(include ${catkin_INCLUDE_DIRS})

# add_library(bundle_manager src/matlab_listener.cpp src/mace_listener.cpp)
# set_target_properties(bundle_manager PROPERTIES LINKER_LANGUAGE CXX)
# add_dependencies(bundle_manager bundle_manager_generate_messages_cpp)
