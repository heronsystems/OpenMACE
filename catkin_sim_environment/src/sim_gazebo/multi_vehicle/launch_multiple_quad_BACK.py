import subprocess
import sys
from sys import argv
import rospkg
import time
import argparse
import json

if __name__ == '__main__':
    # Parse command line arguments:
    parser = argparse.ArgumentParser()
    parser.add_argument('fp', type=str, help='Relative file path to the launch script JSON configuration file. (e.g.: ./config_2vehicles_kinect.json)')
    args = parser.parse_args()
    with open(args.fp, 'r') as in_file:
        args = json.load(in_file)

    
    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    world_name = rospack.get_path('sim_gazebo') + "/worlds/" + args['worldName'] + ".world"

    # Define a terminal and add the world launch to the first tab:
    terminal = ['gnome-terminal']
    terminal.extend(['--tab', '--command', '''
        bash -c '
        roslaunch sim_gazebo start_world.launch paused:={} gui:={} world_name:={} octomap_rviz:={}
        '
    '''.format(args['pause'], args['gui'], args['worldName'], args['octomap']) % locals()])

    # Loop over vehicle ID/sensor array to extend the command to launch each vehicle
    for key, value in args["vehicles"].items():
        print("____________ Launch quad with Vehicle ID = {} ____________".format(key))
        print("================= with sensor = {} =================".format(value))
        # is_fat = True
        # state = "fat" if is_fat else "not fat"
        add_sensors = args["addSensors"] if value != "" else False
        terminal.extend(['--tab', '--command', '''
            bash -c '
            sleep 3
            roslaunch -v sim_gazebo multi_basic_quadrotor.launch vehicle_id:={} add_sensors:={} model_name:={} x:={}
            '
        '''.format(key, add_sensors, value, key) % locals()])


    # Execute the command:
    subprocess.call(terminal)

