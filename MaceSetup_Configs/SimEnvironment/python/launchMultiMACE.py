#!/usr/bin/env python
from os import system
import subprocess
import sys
from sys import argv
import time
import argparse
import json
from generateVehicleXML import generateVehicleXML
from generateGroundXML import generateGroundXML
from createLogDir import createLogDir


if __name__ == '__main__':
    # Parse command line arguments:
    parser = argparse.ArgumentParser()
    parser.add_argument('fp', type=str, help='Relative file path to the launch script JSON configuration file. (e.g.: ./config_1_vehicle.json)')
    config = parser.parse_args()
    with open(config.fp, 'r') as in_file:
        config = json.load(in_file)

    vehicles = config['vehicles']

    # Create log directory:
    relativeLogDir = createLogDir()

    # Create ground instance xml:
    groundXMLFile = generateGroundXML(config['GUIInit']['guiHostAddress'], "16000", relativeLogDir)

    ## Options for {vehicle_type}: ArduCopter|AntennaTracker|APMrover2|ArduSub|ArduPlane
    ## Options for {frame_type} depend on vehicle type. For example, for ArduCopter: octa-quad|tri|singlecopter|gazebo-iris|calibration|hexa|heli|+|heli-compound|dodeca-hexa|heli-dual|coaxcopter|X|quad|y6|IrisRos|octa

    # Loop over vehicle ID/sensor array to extend the command to launch each vehicle according to their type
    if len(vehicles.items()) > 0:
		system('tmux new -d -s vehicles')
		groundWindowName = "GroundInstance"
		system('tmux new-window -n {}'.format(groundWindowName))

		for key, value in vehicles.items():
			print("____________ Launch MACE with ID = {} ____________".format(key))

			# Generate XML (write to directory corresponding to test number or datetime):
			#	- CALL generateMACEXML.py with ethernet port number and listen port number as arguments
			vehicleID = int(key)
			elPort = 16000 + vehicleID
			sitlPort = 15000 + vehicleID
			xmlFile = generateVehicleXML(vehicleID, elPort, sitlPort, relativeLogDir)

			print(xmlFile)

			if value["simulated"] == True:
				time.sleep(1)
				windowName = '{}_{}'.format(value["type"], key)
				system('tmux new-window -n {}'.format(windowName))
				system('tmux send-keys -t {} ENTER ENTER ENTER'.format(windowName))
				system('tmux send-keys -t {} "cd $MACE_ROOT" ENTER'.format(windowName))
				system('tmux send-keys -t {} "./bin/MACE {}" ENTER'.format(windowName, xmlFile))
				#system('tmux send-keys -t {} "./bin/MACE" ENTER'.format(windowName))

		# Start ground instance:
		system('tmux send-keys -t {} ENTER ENTER ENTER'.format(groundWindowName))
		system('tmux send-keys -t {} "cd $MACE_ROOT" ENTER'.format(groundWindowName))
		system('tmux send-keys -t {} "./bin/MACE {}" ENTER'.format(groundWindowName, groundXMLFile))


    # Attach to tmux:
    system('tmux a')
