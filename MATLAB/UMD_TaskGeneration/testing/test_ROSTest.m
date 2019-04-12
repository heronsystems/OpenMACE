% Clear any previously running ROS nodes:
clear
rosshutdown

% Initialize ROS:
setenv('ROS_MASTER_URI','http://10.104.195.179:11311') % ROS Core location
setenv('ROS_IP','10.104.195.179') % MATLAB location
rosinit

% List ROS topics:
rostopic list

% Set up subscribers:
positionSub = rossubscriber('/MACE/UPDATE_POSITION', @positionCallback, 'BufferSize', 10);
%attitudeSub = rossubscriber('/MACE/UPDATE_ATTITUDE', @attitudeCallback, 'BufferSize', 10);
%batterySub = rossubscriber('/MACE/UPDATE_BATTERY', @batteryCallback, 'BufferSize', 10);
gpsSub = rossubscriber('/MACE/UPDATE_GPS', @gpsCallback, 'BufferSize', 10);
%heartbeatSub = rossubscriber('/MACE/UPDATE_HEARTBEAT', @heartbeatCallback, 'BufferSize', 10);
%vehicleTargetSub = rossubscriber('/MACE/TARGET_STATUS',@vehicleTargetCallback, 'BufferSize', 10);


% Set up service clients:
armClient = rossvcclient('command_arm');
takeoffClient = rossvcclient('command_takeoff');
landClient = rossvcclient('command_land');
datumClient = rossvcclient('command_datum');
waypointClient = rossvcclient('command_waypoint');

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
armRequest.VehicleID = 2; % Vehicle ID
armRequest.CommandID = 1; % TODO: Set command ID enum in MACE
armRequest.ArmCmd = true; % True to ARM throttle, False to DISARM

% Setup Vehicle takeoff command:
takeoffRequest = rosmessage(takeoffClient);
takeoffRequest.Timestamp = rostime('now');
takeoffRequest.VehicleID = 2; % Vehicle ID
takeoffRequest.CommandID = 2; % TODO: Set command ID enum in MACE
takeoffRequest.TakeoffAlt = 10; % Takeoff altitude
% If you don't set lat/lon (or set them to 0.0), it will takeoff in current position
% takeoffRequest.LatitudeDeg = 0.0; % If 0.0, takeoff where you currently are
% takeoffRequest.LongitudeDeg = 0.0; % If 0.0, takeoff where you currently are

% Setup Waypoint command :
waypointRequest = rosmessage(waypointClient);
waypointRequest.Timestamp = rostime('now');
waypointRequest.VehicleID = 2; % Vehicle ID
waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
waypointRequest.Northing = 10; % Relative northing position to Datum
waypointRequest.Easting = 10; % Relative easting position to Datum
waypointRequest.Altitude = 10;

% Setup Land command:
landRequest = rosmessage(landClient);
landRequest.Timestamp = rostime('now');
landRequest.VehicleID = 2; % Vehicle ID
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

disp('Call arm command');
armResponse = call(armClient, armRequest, 'Timeout', 5);

% For this test, just wait 3 seconds before issuing takeoff command:
pause(8);

disp('Call takeoff command');
takeoffResponse = call(takeoffClient, takeoffRequest, 'Timeout', 5);

% For this test, just wait 10 seconds before issuing waypoint command (giving vehicle time to reach altitude):
pause(10);

disp('Call waypoint command');
waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 5);

% For this test, just wait 20 seconds before issuing land command (giving vehicle time to reach waypoint):
pause(20);

disp('Call land command');
landResponse = call(landClient, landRequest, 'Timeout', 5);

