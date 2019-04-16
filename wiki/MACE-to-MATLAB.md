# Table of Contents
- [Introduction](#intro)
- [Setup](#setup)
  - [Prerequisites](#prereqs)
  - [MACE](#setup-mace)
  - [MATLAB](#setup-matlab)
- [Run steps](#run-steps)
  - [Running MACE](#run-mace)
  - [Running MATLAB](#run-matlab)

# <a name="intro"></a> Introduction
MACE provides a way to rapidly stand up multi-vehicle test beds to expose swarming capabilities to researchers with a lower barrier to entry than typical swarming architectures. The modular software architecture allows researchers to focus on their area of interest while still benefiting from previously developed robotic capabilities without having to configure those capabilities from the ground up.

A common development environment for researchers is [MATLAB](https://www.mathworks.com/products/matlab.html). MATLAB is a scripting environment that provides easy to use functionality for linear algebra, tools for data visualization, and much more. MACE as currently built allows users to integrate unmanned systems with MATLAB through a [ROS](http://www.ros.org/) wrapper that exposes common commands and data from simulated or hardware-in-the-loop vehicles. This page walks you through how to set up MACE with MATLAB.

# <a name="setup"></a> Setup
## <a name="setup-prereqs"></a> Prerequisites
This test assumes that you have installed MACE and the supporting libraries. Due to the fact that ROS is currently only stable on Linux distributions, this test assumes that it will be run on a Linux distribution with ROS installed. If you have yet to do so, install MACE by following the steps outlined on the following page:

- [Linux Installation](https://github.com/heronsystems/OpenMACE/wiki/Linux-Installation)

Once MACE is installed, make sure to build the MACE ROS simulation environment. Follow the steps outlined on the [ROS MACE Setup](https://github.com/heronsystems/OpenMACE/wiki/ROS-MACE-Setup) page. Install ROS and build the `catkin_sim_environment` as described.

It is also assumed that you have access to a MATLAB license with the [Robotics System Toolbox](https://www.mathworks.com/hardware-support/robot-operating-system.html). The Robotics systems toolbox provides support for ROS in MATLAB. In addition to the toolbox, you must install a support package for MATLAB in order to generate custom messages required by the ROS <-> MACE interface. Instructions for installing the MATLAB support packages can be found here: [Install Robotics System Toolbox Add-Ons](https://www.mathworks.com/help/robotics/ug/install-robotics-system-toolbox-support-packages.html).

**NOTE: If the installation of the add ons fails, follow the instructions on this [Bug Report](https://www.mathworks.com/support/bugreports/1741173) to fix the issue.

For this test, we are also going to be using a simulated Ardupilot vehicle. Follow the steps on the [Ardupilot Simulation](https://github.com/heronsystems/OpenMACE/wiki/ArduPilot-Simulation) for installation and setup instructions.

## <a name="setup-mace"></a> MACE setup
For this test, we want to launch MACE with a Vehicle Comms module for a simulated vehicle as well as a ROS module. Open the default MACE configuration file (`MACE/MaceSetup_Configs/Default.xml`) and replace the contents with the following XML configuration:

```xml
<?xml version="1.0" encoding="utf-8"?>
<ModuleConfigurations MaceInstance="1">
  <Module Class="VehicleComms" Type="Ardupilot">
    <Parameter Name="ProtocolParameters">
      <Parameter Name="Name">Mavlink</Parameter>
      <Parameter Name="Version">V2</Parameter>
    </Parameter>
	<Parameter Name="UDPParameters">
      <Parameter Name="ListenAddress">**YOUR_IP_HERE**</Parameter>
      <Parameter Name="ListenPortNumber">14551</Parameter>
    </Parameter>
    <Parameter Name="ModuleParameters">
      <Parameter Name="AirborneInstance">false</Parameter>
    </Parameter>
  </Module>

  <Module Class='ROS' Type='OFFSET_Auctioneer'>
      <Parameter Name='ModuleParameters'>
        <Parameter Name='AirborneInstance'>false</Parameter>
      </Parameter>
  </Module>

</ModuleConfigurations>
```

Make sure to change `**YOUR_IP_HERE**` to the current IP address of your machine.

## <a name="setup-matlab"></a> MATLAB setup
The ROS/MACE interface requires some custom ROS messages, that are defined in the `catkin_sim_environment` directory of MACE. The steps above to build the `catkin_sim_environment` are necessary for the MACE side of the ROS interface. However, we still need to generate MATLAB specific versions of the custom ROS messages. To do so, we will use the Robotics Toolbox Add on that was installed in the [Prerequisites](#prereqs) section.

Open MATLAB and make sure to add `%MACE_ROOT/catkin_sim_environment` and all of its sub-directories to the MATLAB path. You can do this by navigating to `%MACE_ROOT` in your MATLAB window, right clicking on `catkin_sim_environment`, and selecting "Add to Path -> Selected Floders and Subfolders".

Once those directories are in the MATLAB path, invoke the `rosgenmsg` function by first defining a top-level folder path, then calling `rosgenmsg`:

```matlab
folderpath = 'PATH/TO/MACE/ROOT/catkin_sim_environment/src';
rosgenmsg(folderpath)
```
This will search the folderpath for ROS packages and generate any ROS messages, services, or actions contained within them. For this test, we are concerned with the `mace_matlab` package. Once the `rosgenmsg` command finishes, follow the instructions in the MATLAB terminal to finish the installation of the custom ROS messages.

# <a name="run-steps"></a> Run steps
Before you run this test, you will need to start a `roscore`. To do so, simply open a terminal and first set the **ROS_MASTER_URI** and **ROS_IP** environment variables. Then, run a `roscore`:

```
$ export ROS_MASTER_URI=http://**YOUR_ROS_IP_HERE**:11311
$ export ROS_IP=**YOUR_ROS_IP_HERE**
$ roscore
```
Make sure you set the **ROS_MASTER_URI** to the location where the `roscore` is located. The **ROS_IP** here is where the local ROS node is running, which in this case is the same IP as the **ROS_MASTER_URI**.

## <a name="run-mace"></a> Running MACE
To start MACE, navigate to the `%MACE_ROOT/bin` directory and run `./MACE`:

```
$ cd %MACE_ROOT/bin
$ ./MACE
```
This will run MACE and load the `Default.xml` file we modified in the [setup steps](#setup-mace) above.

## <a name="run-sitl"></a> Running Ardupilot SITL
To start a simulated vehicle, navigate to the directory where you installed the simulator and run the following command:

```
$ cd <PATH>/<TO>/<ARDUPILOT>/ArduCopter
$ sim_vehicle.py --custom-location=37.890425,-76.815610,0,180 --out=udp:**YOUR_IP_HERE**:14551 --instance=1 --vehicle=ArduCopter --frame=quad --console
```
Make sure to replace `**YOUR_IP_HERE**` in the above command with your current IP address, as well as `<PATH>/<TO>/<ARDUPILOT>` to the path where you installed ardupilot. A vehicle will launch and you should see the MACE terminal respond with actions to sync with the vehicle.

## <a name="run-matlab"></a> Running MATLAB
To run MATLAB, we want to set up some subscribers for MATLAB to listen to messages coming from MACE. Add a function file called `positionCallback.m` and put the following code inside the file:

```matlab
function [ output_args ] = positionCallback( subscriber, msg )
     disp('UPDATE_POSITION Callback fired')
     msg
end
```

This will be the callback to the `UPDATE_POSITION` message output from MACE, and it will display that it has been called, as well as a printout of the message received. **Make sure to make a duplicate M-file for each of the following:**

- attitudeCallback
- batteryCallback
- heartbeatCallback
- gpsCallback
- vehicleTargetCallback

Below is a simple MATLAB script that will setup subscribers to each custom ROS message that MACE can output. In addition, a service client is set up for each command available (implemented as ROS services). At the top of this script, notice the `setenv` commands, where we set the **ROS_MASTER_URI** and **ROS_IP** as we did when running the `roscore` above. In this case, the **ROS_MASTER_URI** IP is again the IP where the `roscore` is running. However, the **ROS_IP** in this case is the IP address where MATLAB is located (i.e. where the local ROS node is running).

**Note that at the time of this writing, there is no way to monitor the status of a command without writing a custom check. For this simple test, we simply wait a predetermined amount of time to allow takeoff, waypoint, and land commands to execute properly.**

```matlab
clear
rosshutdown

% Initialize ROS:
setenv('ROS_MASTER_URI','http://**YOUR_ROS_IP_HERE**:11311') % ROS Core location
setenv('ROS_IP','**YOUR_MATLAB_IP_HERE') % MATLAB location
rosinit

% List ROS topics:
rostopic list

% Set up subscribers:
positionSub = rossubscriber('/MACE/UPDATE_POSITION', @positionCallback, 'BufferSize', 10);
attitudeSub = rossubscriber('/MACE/UPDATE_ATTITUDE', @attitudeCallback, 'BufferSize', 10);
batterySub = rossubscriber('/MACE/UPDATE_BATTERY', @batteryCallback, 'BufferSize', 10);
gpsSub = rossubscriber('/MACE/UPDATE_GPS', @gpsCallback, 'BufferSize', 10);
heartbeatSub = rossubscriber('/MACE/UPDATE_HEARTBEAT', @heartbeatCallback, 'BufferSize', 10);
vehicleTargetSub = rossubscriber('/MACE/TARGET_STATUS',@vehicleTargetCallback, 'BufferSize', 10);

% Set up service clients:
datumClient = rossvcclient('command_datum');
armClient = rossvcclient('command_arm');
takeoffClient = rossvcclient('command_takeoff');
landClient = rossvcclient('command_land');
waypointClient = rossvcclient('command_wpt');

% Example workflow:
%   1) Set datum
%   2) Arm vehicle
%   3) Takeoff vehicle
%   4) Issue waypoint command after altitude achieved
%   5) Land vehicle after waypoint achieved

% Setup datum command:
datumRequest = rosmessage(datumClient);
datumRequest.Timestamp = rostime('now');
datumRequest.VehicleID = 0; % Not necessary for this
datumRequest.CommandID = 0; % TODO: Set command ID enum in MACE
datumRequest.LatitudeDeg = 37.889246;
datumRequest.LongitudeDeg = -76.814084;

% Setup Arm vehicle command:
armRequest = rosmessage(armClient);
armRequest.Timestamp = rostime('now');
armRequest.VehicleID = 1; % Vehicle ID
armRequest.CommandID = 1; % TODO: Set command ID enum in MACE
armRequest.ArmCmd = true; % True to ARM throttle, False to DISARM

% Setup Vehicle takeoff command:
takeoffRequest = rosmessage(takeoffClient);
takeoffRequest.Timestamp = rostime('now');
takeoffRequest.VehicleID = 1; % Vehicle ID
takeoffRequest.CommandID = 2; % TODO: Set command ID enum in MACE
takeoffRequest.TakeoffAlt = 10; % Takeoff altitude
% % If you don't set lat/lon (or set them to 0.0), it will takeoff in current position
% takeoffRequest.LatitudeDeg = 0.0; % If 0.0, takeoff where you currently are
% takeoffRequest.LongitudeDeg = 0.0; % If 0.0, takeoff where you currently are

% Setup Waypoint command :
waypointRequest = rosmessage(waypointClient);
waypointRequest.Timestamp = rostime('now');
waypointRequest.VehicleID = 1; % Vehicle ID
waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
waypointRequest.Northing = 10;
waypointRequest.Easting = 10;
waypointRequest.Altitude = 10;

% Setup Land command:
landRequest = rosmessage(landClient);
landRequest.Timestamp = rostime('now');
landRequest.VehicleID = 1; % Vehicle ID
landRequest.CommandID = 4; % TODO: Set command ID enum in MACE

datumResponse = false;
armResponse = false;
takeoffResponse = false;
waypointResponse = false;
landResponse = false;


disp('Call set datum command');
datumResponse = call(datumClient, datumRequest, 'Timeout', 5);

% For this test, just wait 5 seconds before issuing arm command:
pause(5);

% disp('Call arm command');
armResponse = call(armClient, armRequest, 'Timeout', 5);

% For this test, just wait 3 seconds before issuing takeoff command:
pause(3);

disp('Call takeoff command');
takeoffResponse = call(takeofClient, takeoffRequest, 'Timeout', 5);

% For this test, just wait 10 seconds before issuing waypoint command (giving vehicle time to reach altitude):
pause(10);

disp('Call waypoint command');
waypointResponse = call(datumClient, datumRequest, 'Timeout', 5);

% For this test, just wait 20 seconds before issuing land command (giving vehicle time to reach waypoint):
pause(20);

disp('Call land command');
landResponse = call(landClient, landRequest, 'Timeout', 5);
```
