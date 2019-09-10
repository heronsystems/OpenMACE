#!/bin/bash
# This script launches any number of vehicles with IDs provided by the user. Script arguments are as folows:

# store arguments in a special array 
args=("$@") 
# get number of elements 
ELEMENTS=${#args[@]} 

# Default values:
idList="" # Formatted as: idArr:=[1,2,5] in command. No spaces, comma separated
sensorList=$idList # Formatted as: sensorArr:=[kinect,kinect,kinect] in command. No spaces, comma separated
addSensors="true"
worldName="turtlebot_playground"
gui="false"
paused="false"
rvizRun="false"
#rvizConfig="-d $(find sim_gazebo)/rviz/default.rviz"

# Parse command line arguments into variables:
for (( i=0;i<$ELEMENTS;i++)); do 
    IFS='=' read -r var value <<< "${args[${i}]}"
    argCall="${var:0:${#var}-1}"
    #echo "ARG #" $i
    #echo "ARG NAME:" $argCall
    #echo "ARG VALUE:" $value

    if [ $argCall = "idArr" ]
	then
	    idList=${value:1:${#value}-2}
    fi
    if [ $argCall = "sensorArr" ]
	then
	    sensorList=${value:1:${#value}-2}
    fi
    if [ $argCall = "add_sensors" ]
	then
	    addSensors=$value
    fi
    if [ $argCall = "world_name" ]
	then
	    worldName=$value
    fi
    if [ $argCall = "gui" ]
	then
	    gui=$value
    fi
    if [ $argCall = "paused" ]
	then
	    paused=$value
    fi
    if [ $argCall = "rviz_run" ]
	then
	    rvizRun=$value
    fi
    if [ $argCall = "rviz_config" ]
	then
	    rvizConfig=$value
    fi 
    # Put new arguments here...
done

# Set arrays:
idArray=(${idList//,/ })
sensorArray=(${sensorList//,/ })


# Before we go any further, if we are adding sensors, first check if the arrays are the same size
if [ $addSensors = "true" ]
	then
	    # Check if idArr and sensorArr are the same length. if not, break out and tell user
	    if [ ${#idArray[@]} != ${#sensorArray[@]} ]
		then
		    echo " ** ID Array and Sensor Array are not the same length. Make sure each vehicle ID has a corresponding sensor model. Note that the sensor can be left blank ** "
		    exit 1
	    fi
fi

# Launch roscore:
#xterm -title "roscore" -hold -e roscore &


# Launch world:
source $MACE_ROOT/catkin_sim_environment/devel/setup.bash
world_name=$(rospack find sim_gazebo)/worlds/$worldName.world
xterm -title "World" -hold -e roslaunch sim_gazebo start_world.launch world_name:=$world_name paused:=$paused gui:=$gui &


# Loop over vehicles and sensors to spawn each vehicle
for i in "${!idArray[@]}"
do
	echo " ____________ Launch quad with Vehicle ID = ${idArray[i]} ____________ "
	echo " ================= with sensor = ${sensorArray[i]} ================= "
	
	if [ i != ${#idArray[@]} ]
	    then
		xterm -title "Vehicle "${idArray[i]} -hold -e roslaunch sim_gazebo multi_basic_quadrotor.launch vehicle_id:=${idArray[i]} add_sensors:=$addSensors model_name:=${sensorArray[i]} x:=$i &
	else
		# WTF is this? Why did I have the same command in the if/else statement?
		xterm -title "Vehicle "${idArray[i]} -hold -e roslaunch sim_gazebo multi_basic_quadrotor.launch vehicle_id:=${idArray[i]} add_sensors:=$addSensors model_name:=${sensorArray[i]} x:=$i
	fi
done

# Launch RVIZ, if applicable:



