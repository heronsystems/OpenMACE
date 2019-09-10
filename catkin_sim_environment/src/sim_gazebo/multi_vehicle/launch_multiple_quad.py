from os import system
from sys import argv
import rospkg
import time
import argparse
import json

if __name__ == '__main__':
    # Parse command line arguments:
    parser = argparse.ArgumentParser()
    parser.add_argument('fp', type=str, help='Relative file path to the launch script JSON configuration file. (e.g.: ./config_2vehicles_kinect.json)')
    config = parser.parse_args()
    with open(config.fp, 'r') as in_file:
        config = json.load(in_file)

    rosArgs = config['ROSParams']
    vehicles = config['vehicles']

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    world_name = rospack.get_path('sim_gazebo') + "/worlds/" + rosArgs['worldName'] + ".world"

    # Define a terminal and add the world launch to the first tab:
    system('tmux new -d -s ros_vehicles')
    system('tmux new-window -n {}'.format("ros_world"))
    system('tmux send-keys -t ros_world "roslaunch sim_gazebo start_world.launch paused:={} gui:={} world_name:={} octomap_rviz:={}" ENTER'.format(rosArgs['pause'], rosArgs['gui'], rosArgs['worldName'], rosArgs['octomap']))

    # Loop over vehicle ID/sensor array to extend the command to launch each vehicle
    for key, value in vehicles.items():
        print("____________ Launch quad with Vehicle ID = {} ____________".format(key))
        print("================= with sensor = {} =================".format(value))
        time.sleep(1)
        windowName = '{}_{}'.format(value["type"], key)
        system('tmux new-window -n {}'.format(windowName))
        add_sensors = value["enableSensor"] if value != "" else False

        system('tmux send-keys -t {} "roslaunch -v sim_gazebo multi_basic_quadrotor.launch vehicle_id:={} add_sensors:={} model_name:={} x:={}" ENTER'.format(windowName, key, add_sensors, value["sensor"], key))


    # Attach to tmux:
    system('tmux a')