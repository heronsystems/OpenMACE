First remove the bundle_manager folder from catkin_ws_environment when compiling ROS_MACE messages. (It will fail due to lack of CMakeLists.txt). Then add the folder back and run rosmsg().
