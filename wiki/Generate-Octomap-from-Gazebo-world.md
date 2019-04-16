The Gazebo plugin used to generate the octomap .bt file was ported from the [rotors_simulator](https://github.com/ethz-asl/rotors_simulator) package. Since it was the only piece of the package that was being used, it was moved into the MACE build tree to remove a dependency on the `rotors_simulator` libraries.

## Prerequisites
The only prerequisites are that ROS is installed (see [HERE](http://wiki.ros.org/kinetic/Installation/Ubuntu) for install steps) and the catkin build environment is set up (see the Build Steps [HERE](https://github.com/heronsystems/OpenMACE/wiki/ROS-MACE-Setup#building)).

## Configuring .world file
To use the plugin, you need to edit your desired .world file to recognize the plugin. Simply open your .world file in a text editor and add the following line just before the final `<world>` tag (i.e. in between the `<world>` tags):
```
<plugin name='gazebo_octomap' filename='libBuildOctomapPlugin.so'/>
```

## Calling Octomap Service
To build an octomap .bt file, open three separate terminals:

```
In one terminal:
$ roscore

In second terminal:
$ rosrun gazebo_ros gazebo <your_world_file>.world
**NOTE: replace <your_world_file> with the filename of the world you wish to build a map of**

In third terminal, once Gazebo has loaded the world above:
$ rosservice call /world/build_octomap '{bounding_box_origin: {x: 0, y: 0, z: 15}, bounding_box_lengths: {x: 30, y: 30, z: 30}, leaf_size: 0.5, filename: output_filename.bt}'
```

Note that the above rosservice call has a few adjustable variables. The bounding box origin can be set as desired (in meters) as well as the bounding box lengths (in meters) relative to the bounding box origin. The bounding box lengths are done in both (+/-) directions relative to the origin. For example, in the `rosservice` call above, from `(0, 0, 0)`, our bounding box will start at **-15 meters** and end at **+15 meters** in the X and Y directions. In the Z direction, we will start at **0 meters** and end at **30 meters**. *To help with later conversions in the MACE path planning module, we typically want to set our Z origin value and bounding box length such that every object in our world starts at 0 meters. For example, if we have a 30 meter tall building, your origin should be at 15 meters and the bounding box length should be 30 meters.*

You can also set the octomap leaf size as desired, in meters. Specify the output filename, which will be generated in the directory where you run the `rosservice` call.

When you execute the rosservice call, you should see an indication that the service is running in the second terminal (i.e. the terminal that Gazebo was started in). The text should say something similar to `Rasterizing world and checking collisions` or similar. **Note that depending on the world size and obstacles in the world, this command may take a while.**

## Visualizing the generated octomap
You can visualize the generated octomap using [Octovis](http://wiki.ros.org/octovis). To get Octovis, simply run `sudo apt-get install ros-<distro>-octovis`, where `<distro>` can be replaced by your appropriate ROS distribution (i.e. kinetic, lunar, etc.). Once installed, navigate to the directory where your generated .bt file is and run:
```
octovis <filname>.bt
```
Note that larger octomaps are harder to visualize and performance when using Octovis may suffer.