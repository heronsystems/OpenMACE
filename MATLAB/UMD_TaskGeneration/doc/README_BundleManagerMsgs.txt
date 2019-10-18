First remove the bundle_manager folder from catkin_ws_environment when compiling ROS_MACE messages. (It will fail due to lack of CMakeLists.txt). Then add the folder back and run rosmsg().



-----


To enable the new bundle msg and serv, simply run the following two lines in MATLAB:
folderpath = 'PATH/TO/MACE/ROOT/catkin_sim_environment/src';
rosgenmsg(folderpath);

Follow the instructions listed in the command window and restart MATLAB. Now, you need to make sure you can see the following three items after typing 'rosmsg list' followed by an ENTER

bundle_manager/BUNDLE_CONTENT                                  
bundle_manager/BUNDLE_REQUESTRequest                          
bundle_manager/BUNDLE_REQUESTResponse
