
% Clear any previously running ROS nodes:
clc
close all
clear all

%% Setup Environmental Conditions
should_setDatum = false;
should_setHome = true;
should_arm = false;
should_takeoff = false;

test_PositionSequence_Local = false;
test_VelocitySequence = false;
test_PositionSequence_Global = false;
test_AttitudeSequence_Euler = false;
test_WaypointCommand = false;

should_land = false;

%% Initialize the MATLAB/ROS Environment
% Initialize ROS:
% setenv('ROS_MASTER_URI','http://192.168.1.20:11311') % ROS Core location
% setenv('ROS_IP','192.168.1.166') % MATLAB location
setenv('ROS_MASTER_URI','http://127.0.0.1:11311') % ROS Core location
setenv('ROS_IP','127.0.0.1') % MATLAB location
rosinit

% List ROS topics:
rostopic list

% Set up subscribers:
localPositionSub = rossubscriber('/MACE/UPDATE_LOCAL_POSITION', @localPositionCallback, 'BufferSize', 10);
geodeticPositionSub = rossubscriber('/MACE/UPDATE_GEODETIC_POSITION', @geodeticPositionCallback, 'BufferSize', 10);

attitudeSub = rossubscriber('/MACE/UPDATE_ATTITUDE', @attitudeCallback, 'BufferSize', 10);
batterySub = rossubscriber('/MACE/UPDATE_BATTERY', @batteryCallback, 'BufferSize', 10);
gpsSub = rossubscriber('/MACE/UPDATE_GPS', @gpsCallback, 'BufferSize', 10);
heartbeatSub = rossubscriber('/MACE/UPDATE_HEARTBEAT', @heartbeatCallback, 'BufferSize', 10);
vehicleTargetSub = rossubscriber('/MACE/TARGET_STATUS',@vehicleTargetCallback, 'BufferSize', 10);


% Set up service clients:
armClient = rossvcclient('command_arm');
takeoffClient = rossvcclient('command_takeoff');
landClient = rossvcclient('command_land');
datumClient = rossvcclient('command_datum');
homeClient = rossvcclient('command_home');
dynamicTargetClient_Kinematic = rossvcclient('command_dynamic_target_kinematic');
dynamicTargetClient_EOrientation = rossvcclient('command_dynamic_target_euler');
dynamicTargetClient_QOrientation = rossvcclient('command_dynamic_target_quat');

waypointClient = rossvcclient('command_waypoint');


%% Workflow
% Example workflow:
%   1) Set datum
%   2) Arm vehicle
%   3) Takeoff vehicle
%   4) Issue waypoint command after altitude achieved
%   5) Land vehicle after waypoint achieved

%% Execute Datum Sequence
if should_setDatum
    disp('Call set datum command');
    
    % Setup datum command:
    datumRequest = rosmessage(datumClient);
    datumRequest.Timestamp = rostime('now');
    datumRequest.VehicleID = 0; % Not necessary for this
    datumRequest.CommandID = 0; % TODO: Set command ID enum in MACE
    datumRequest.LatitudeDeg = -35.3631970;
    datumRequest.LongitudeDeg = 149.1653205;
    datumRequest.AltitudeMsl = 584.0;
    
    datumResponse = call(datumClient, datumRequest, 'Timeout', 5);
    pause(5);
end



%% Execute Datum Sequence
if should_setHome
    disp('Call set home command');

    % Setup datum command:
    homeRequest = rosmessage(homeClient);
    homeRequest.Timestamp = rostime('now');
    homeRequest.VehicleID = 0; % Not necessary for this
    homeRequest.SetToCurrent = true; % TODO: Set command ID enum in MACE
    
    datumResponse = call(homeClient, homeRequest, 'Timeout', 5);
    pause(5);
end
%% Execute Arm Command

if should_arm
    disp('Call arm command');
    
    % Setup Arm vehicle command:
    armRequest = rosmessage(armClient);
    armRequest.Timestamp = rostime('now');
    armRequest.VehicleID = 1; % Vehicle ID
    armRequest.CommandID = 1; % TODO: Set command ID enum in MACE
    armRequest.ArmCmd = true; % True to ARM throttle, False to DISARM
    
    armResponse = call(armClient, armRequest, 'Timeout', 5);
    pause(2);
end

%% Execute Takeoff Command

if should_takeoff
    disp('Call takeoff command');
    
    % Setup Vehicle takeoff command:
    takeoffRequest = rosmessage(takeoffClient);
    takeoffRequest.Timestamp = rostime('now');
    takeoffRequest.VehicleID = 1; % Vehicle ID
    takeoffRequest.CommandID = 2; % TODO: Set command ID enum in MACE
    takeoffRequest.TakeoffAlt = 10; % Takeoff altitude
    % If you don't set lat/lon (or set them to 0.0), it will takeoff in current position
    % takeoffRequest.LatitudeDeg = 0.0; % If 0.0, takeoff where you currently are
    % takeoffRequest.LongitudeDeg = 0.0; % If 0.0, takeoff where you currently are
    
    takeoffResponse = call(takeoffClient, takeoffRequest, 'Timeout', 10);
    pause(20);
    
    dynamicTargetRequest = rosmessage(dynamicTargetClient_Kinematic);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.CoordinateFrame = 3; %3 is global relative alt
    dynamicTargetRequest.XP = 149.1653205; %longitude is in the X position
    dynamicTargetRequest.YP = -35.3631970; %latitude is in the Y position
    dynamicTargetRequest.ZP = 20;
    dynamicTargetRequest.Bitmask = 65528; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(20);
end


%% Execute a square pattern with local coordinates

if test_PositionSequence_Local
    local_position_test();
end

%% Execute a series of velocity commands

if test_VelocitySequence
    dynamicTargetRequest = rosmessage(dynamicTargetClient_Kinematic);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.CoordinateFrame = 11;
    dynamicTargetRequest.XP = 0;
    dynamicTargetRequest.YP = 0;
    dynamicTargetRequest.ZP = -20;
    dynamicTargetRequest.XV = 0;
    dynamicTargetRequest.YV = 0;
    dynamicTargetRequest.ZV = 0;
    dynamicTargetRequest.Bitmask = 65479; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    
    %waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 5);
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(2);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XV = 0;
    dynamicTargetRequest.YV = 2;
    dynamicTargetRequest.ZV = 0;
    dynamicTargetRequest.Yaw = 1.5708;
    dynamicTargetRequest.Bitmask = 64455; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XV = 2;
    dynamicTargetRequest.YV = 0;
    dynamicTargetRequest.ZV = 0;
    dynamicTargetRequest.Yaw = 0;
    dynamicTargetRequest.Bitmask = 64455; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XV = 0;
    dynamicTargetRequest.YV = -2;
    dynamicTargetRequest.ZV = 0;
    dynamicTargetRequest.Yaw = 4.71239;
    
    dynamicTargetRequest.Bitmask = 64455; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XV = -2;
    dynamicTargetRequest.YV = 0;
    dynamicTargetRequest.ZV = 0;
    dynamicTargetRequest.Yaw = 3.14159;
    
    dynamicTargetRequest.Bitmask = 64455; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XV = 0;
    dynamicTargetRequest.YV = 0;
    dynamicTargetRequest.ZV = 0;
    dynamicTargetRequest.Bitmask = 64455; %65528 is for position, 65479 is for velocity, 65472 is position and velocity, 64455 is for velocity and yaw
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(2);
end

%% Execute a square pattern with Global coordinates

if test_PositionSequence_Global
    global_position_test();
end

%% Execute Euler Commands

if test_AttitudeSequence_Euler
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = 0; %longitude is in the X position
    dynamicTargetRequest.Pitch = 0.0; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(5);
    
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = 0.26; %longitude is in the X position
    dynamicTargetRequest.Pitch = 0.0; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(5);
    
    
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = 0.0; %longitude is in the X position
    dynamicTargetRequest.Pitch = 0.0; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(2);
    
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = 0.0; %longitude is in the X position
    dynamicTargetRequest.Pitch = 0.26; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(5);
    
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = 0.0; %longitude is in the X position
    dynamicTargetRequest.Pitch = 0.0; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(2);
    
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = -0.26; %longitude is in the X position
    dynamicTargetRequest.Pitch = 0.0; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(5);
    
    
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = 0.0; %longitude is in the X position
    dynamicTargetRequest.Pitch = 0.0; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(2);
    
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = 0.0; %longitude is in the X position
    dynamicTargetRequest.Pitch = -0.26; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(5);
    
    % Setup orientation command:
    dynamicTargetRequest = rosmessage(dynamicTargetClient_EOrientation);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.Roll = 0.0; %longitude is in the X position
    dynamicTargetRequest.Pitch = 0.0; %latitude is in the Y position
    dynamicTargetRequest.Yaw = 0.0;
    dynamicTargetRequest.Thrust = 0.5;
    dynamicTargetRequest.Bitmask = 63; %127 is for attitude, 191 is for thrust, 63 is attitude and thrust
    disp('Call set desired euler');
    waypointResponse = call(dynamicTargetClient_EOrientation, dynamicTargetRequest, 'Timeout', 5);
    pause(2);
end


%% Execute a Simple Waypoint Test

if test_WaypointCommand
    
    %     Setup Waypoint command :
    waypointRequest = rosmessage(waypointClient);
    waypointRequest.Timestamp = rostime('now');
    waypointRequest.VehicleID = 1; % Vehicle ID
    waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
    waypointRequest.Northing = 10; % Relative northing position to Datum
    waypointRequest.Easting = 10; % Relative easting position to Datum
    waypointRequest.Altitude = 10;
    
    disp('Call waypoint command');
    waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 5);
    
    
end

%% Execute a landing

if should_land
    
    % Setup Land command:
    landRequest = rosmessage(landClient);
    landRequest.Timestamp = rostime('now');
    landRequest.VehicleID = 1; % Vehicle ID
    landRequest.CommandID = 4; % TODO: Set command ID enum in MACE
    
    disp('Call land command');
    landResponse = call(landClient, landRequest, 'Timeout', 5);
    
end
