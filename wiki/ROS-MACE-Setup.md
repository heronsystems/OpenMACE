# ROS Simulation Environment
## Table of Contents
- [Build steps](#building)
- [Run steps](#run)
  - [Running multiple vehicles](#multi)
- [Generating a new configuration](#new-config)

## <a name="building"></a>Build steps
Currently, the only tested ROS distribution with MACE is Indigo and Kinetic. Follow the instructions on the ROS wiki to install:
[Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Follow the default install and use default install paths, as MACE currently checks the default locations for library linkings.

After ROS has installed, in the top level directory of the MACE catkin environment (i.e. `MACE/catkin_sim_environment`), run `catkin_make`. Once the catkin simulation environment is built successfully, add the `setup.bash` file to your `bashrc` environment by running:

```bash
echo "source ~/<path>/<to>/<catkin_sim_environment>/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash
```
Make sure to change `<path>/<to>/<catkin_sim_environment>` to the correct path, pointing to the top level catkin_sim_environment directory. 

If you wish to use ROS with MACE, you will also need to install `octomap` and `tf` libraries. To do so, run the following:

```
$ sudo apt-get install ros-<distro>-octomap*
$ sudo apt-get install ros-<distro>-tf*
```

Make sure to replace `<distro>` with the correct ROS distribution (e.g. indigo, kinetic)

## <a name="run"></a>Run steps
Included in this simulation environment are several pre-made launch files that launch ROS/Gazebo, load various worlds and UAV models, and various sensor models. An example launch command to launch the Gazebo GUI with the `turtlebot_playground` world provided in the repository is as follows:
```bash
roslaunch sim_gazebo start_world.launch paused:=false gui:=true world_name:=turtlebot_playground
```
To launch a simulated quad rotor in the same world with a Kinect sensor, you can run the following:
```bash
roslaunch sim_gazebo basic_quadrotor.launch paused:=false gui:=true vehicle_id:=1 add_sensors:=true model_name:=kinect world_name:=turtlebot_playground
```
The available launch files can be found in the [launch](https://github.com/heronsystems/MACE/tree/master/catkin_sim_environment/src/sim_gazebo/launch) directory.

### <a name="multi"></a>Running multiple vehicles
This simulation environment also supports launching multiple vehicles in the same world. To date, this functionality only pertains to basic quadrotors with default sensors (e.g. kinect, hokuyo, etc.) on Ubuntu operating systems. This script depends on `python-rospkg`, which can be installed using:

```
$ sudo apt-get install python-rospkg
```

To launch multiple vehicles, simply call the `launch_multiple_quad.py` file using Python3 and specifying a configuration file path. For example, from the `catkin_sim_environment/src/sim_gazebo/multi_vehicle/` directory, run:
```bash
python3 launch_multiple_quad.py ./configs/config_1vehicle_hokuyo.json 
```
This will load the simulation parameters from the `config_1vehicle_hokuyo.json` file (in the `/configs` directory). At the time of this writing, the configuration file is as follows:
```json
{
    "GUIInit": {
        "mapCenter": {
            "lat": 37.889231,
            "lng": -76.810302,
            "alt": 0
        },
        "maxZoom": 21,
        "mapZoom": 20
    },
    "ROSParams": {
        "enabled": true,
        "gui": true,
        "pause": false,
        "worldName": "turtlebot_playground",
        "octomap": true
    },
    "MACEParams": {
        "gcsEnabled": true,
        "pathPlanningEnabled": true,
        "externalLinkEnabled": true,
        "id": 1,
        "guiHostAddress": "192.168.1.47",
        "guiSendPort": 1234,
        "guiListenPort": 5678,
        "globalOrigin": {
            "lat": 37.889231,
            "lng": -76.810302
        },
        "octomap": {
            "filename": "test.bt",
            "project2D": true,
            "minRange": 0,
            "maxRange": 9999,
            "occupancyThreshold": 1,
            "probabilityOfHit": 0.7,
            "probabilityOfMiss": 0.4,
            "minThreshold": 0.12,
            "maxThreshold": 0.97
        },
        "externalLink": {
            "portName": "COM7",
            "baudRate": 57600
        },
        "boundaryVertices": [
            {
                "lat": 37.88956019257111,
                "lng": -76.81080847978593
            },
            {
                "lat": 37.88955595899373,
                "lng": -76.8102076649666
            },
            {
                "lat": 37.889335812634876,
                "lng": -76.81060060858728
            }
        ]
    },
    "vehicles": {
        "1": {
            "type": "ArduCopter",
            "frame": "quad",
            "sensor": "hokuyo_utm30lx",
            "enableSensor": true,
            "position": {
                "lat": 37.890425,
                "lng": -76.81191,
                "alt_msl": 0,
                "heading_deg": 240
            },
            "console": false,
            "simulated": true,
            "outputIP": "192.168.1.47",
            "outputPort": 14558,
            "commPort": "COM3"
        }
    }
}
```
This configuration will tell subsequent roslaunch commands to enable the Gazebo GUI, add sensors to each quadrotor, load the `turtlebot_playground` world, and launch 1 vehicle. The vehicle (ID = 1) will have a Hokuyo laser sensor. 

If you wish to add additional vehicles to the configuration, you can modify the `vehicles` dictionary in the config file by adding the vehicle ID and its properties to the `"vehicles"` section of the configuration. An example of the vehicle structure structure is:
```json
"1": {
        "type": "ArduCopter",
        "frame": "quad",
        "sensor": "hokuyo_utm30lx",
        "enableSensor": true,
        "position": {
            "lat": 37.890425,
            "lng": -76.81191,
            "alt_msl": 0,
            "heading_deg": 240
        },
        "console": false,
        "simulated": true,
        "outputIP": "192.168.1.47",
        "outputPort": 14558,
        "commPort": "COM3"
    }
```

**Make sure the vehicle ID's are unique.** Available sensors are:

* hokuyo_utm30lx
* kinect
* asus
* asus_with_hokuyo_utm30lx
* cam
* downward_cam
* all_up

Additionally, you can leave an empty string, which will tell the roslaunch files to not spawn any sensors onboard the vehicle with the corresponding ID. 

## <a name="new-config"></a>Generating a new configuration
As shown above, the configuration file contains a large amount of information that is not relevant to the simulated vehicle. This JSON configuration file is used to not only launch the simulated ROS vehicle, but also to launch a simluated ArduPilot vehicle AND generate the MACE XML configuration file. To make generating this configuration file easier, a GUI application was developed. The GitHub repository is located here: [MACEConfigGenerator](https://github.com/heronsystems/MACEConfigGenerator). Follow the build steps and run steps to generate your own configuration files as required.