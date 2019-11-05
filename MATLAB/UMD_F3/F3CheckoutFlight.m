% Main script for waypoint and speed+heading controlled flights at F3 using 
% MATLAB
% This script is adopted from main_F3CheckoutFlight.m by A. Wolek and S.
% Cheng, Sept. 2018 - Sept. 2019
% 
% S. Cheng, Oct. 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
rosshutdown; % in case there is a leftover instance

% Assumes
% MACE, ROS_MACE, roscore, running 

% Steps
% 1. Update 'ROS_MACE.ip' address in loadParams_testingF3()
% 2. Update agentID to correspond to the Arducopter sim
% 3. Run this script
% (See README.txt for further details)

% Setup ROS_MACE
[runParams, ROS_MACE] = loadParams_cityblocksAtF3();
ROS_MACE = setupF3FlightTestPlot( runParams, ROS_MACE);

% temporary fix to allow plotting with time on ROS message callback
% will be replaced with MACE timestamp when available
global tStart;
tStart = tic;


% % ============= Test: N Quads Takeoff, Wpt Mission, and Land ==============
% ROS_MACE.N = 1;
% %ROS_MACE.operationalAlt = [4 8]; % m
% %ROS_MACE.agentIDs = [1 2]; % m
% ROS_MACE.operationalAlt = [2 3 4 5 6 7]; % m
% ROS_MACE.agentIDs = [1 2 3 4 5 6]; % SYSID_THISMAV on each quadrotor
% 
% ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
% ROS_MACE.wptCoordinator = 'integrated';
% 
% for i = 1:1:length(ROS_MACE.agentIDs)
%     ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
% end
% 
% ROS_MACE = launchROS( ROS_MACE );
% swarmState = sendDatumAndWaitForGPS( ROS_MACE );
% armAndTakeoff( ROS_MACE );
% disp('Press any key to launch waypoint mission...')
% pause;
% 
% % evenly distribute N quads between y from 1m to 11m and -11m to -1m
% temp = linspace(0,20,ROS_MACE.N+2);
% temp = temp(2:end-1);
% yLocation = temp(temp<10)-11;
% yLocation = [yLocation temp(temp>=10)-9];
% 
% wpts = cell(1,ROS_MACE.N);
% 
% for k = 1:ROS_MACE.N
%     if yLocation(k)>0
%         wpts{k} = [11-yLocation(k) yLocation(k);...
%                    -9-yLocation(k) yLocation(k);...
%                    11-yLocation(k) yLocation(k)];
%     else
%         wpts{k} = [11+yLocation(k) yLocation(k);...
%                     -9+yLocation(k) yLocation(k);...
%                    11+yLocation(k) yLocation(k)];
%     end
% end
% % wpts{1} = [5 6;-15 6;10 6;]; % each vector is for a single agent
% % wpts{1} = [5 -6;-15 -6;10 -6];
% 
% 
% captureRadius = 1;% 1.2;
% wptManager( ROS_MACE, wpts, captureRadius);
% 
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );

% ============= Test: 1 quad takeoff, kinematic command, and land =========
ROS_MACE.N = 1;
%ROS_MACE.operationalAlt = [4 8]; % m
%ROS_MACE.agentIDs = [1 2]; % m
ROS_MACE.operationalAlt = [2]; % m
ROS_MACE.agentIDs = [1]; % SYSID_THISMAV on each quadrotor

ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
ROS_MACE.wptCoordinator = 'integrated';

for i = 1:1:length(ROS_MACE.agentIDs)
    ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
end

ROS_MACE = launchROS( ROS_MACE );
swarmState = sendDatumAndWaitForGPS( ROS_MACE );
armAndTakeoff( ROS_MACE );
disp('Press any key to launch the kinematic mission...')
pause;

wpts{1} = [10 -5]; % reference to the waypoint mission that flies closer 

captureRadius = 1;% 1.2;
wptManager( ROS_MACE, wpts, captureRadius);

% == send ==
%     updateWpts( ROS_MACE, wptsDesired, wptIndex );
% Positions are relative to the vehicle�s EKF Origin in NED frame
%
% I.e x=1,y=2,z=3 is 1m North, 2m East and 3m Down from the origin
%
% The EKF origin is the vehicle�s location when it first achieved a good position estimate
%
% Velocity are in NED frame
dynamicTargetRequest = rosmessage(ROS_MACE.kinematicTargetClient);
dynamicTargetRequest.Timestamp = rostime('now');
dynamicTargetRequest.VehicleID = 1; % Vehicle ID
dynamicTargetRequest.CoordinateFrame = 11;
dynamicTargetRequest.XP = 0;
dynamicTargetRequest.YP = 0;
dynamicTargetRequest.ZP = 0;

[NEU_E,NEU_N] = F3toENU(0.1, 0.3);

dynamicTargetRequest.XV = 0;%NEU_N;
dynamicTargetRequest.YV = 0;%NEU_E;
dynamicTargetRequest.ZV = 0;
dynamicTargetRequest.Yaw = pi/4; % in radian
dynamicTargetRequest.YawRate = 0;
dynamicTargetRequest.Bitmask = bin2dec('1111101111000111');%    65479; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
%                                            '1111101111000111'
%waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 5);
fprintf('Going to start location.\n');
waypointResponse = call(ROS_MACE.kinematicTargetClient, dynamicTargetRequest, 'Timeout', 5);

for k =1:4
    msg = ROS_MACE.positionSub.LatestMessage;
    positionCallback(ROS_MACE,msg);
    pause(1);
end
fprintf('start spinning.\n');

dynamicTargetRequest = rosmessage(ROS_MACE.kinematicTargetClient);
dynamicTargetRequest.Timestamp = rostime('now');
dynamicTargetRequest.VehicleID = 1; % Vehicle ID
dynamicTargetRequest.CoordinateFrame = 17; % switch to MAV_FRAME_BODY_OFFSET
dynamicTargetRequest.XP = 0;
dynamicTargetRequest.YP = 0;
dynamicTargetRequest.ZP = 0;

[NEU_E,NEU_N] = F3toENU(0.1, 0.3);

dynamicTargetRequest.XV = 0;%NEU_N;
dynamicTargetRequest.YV = 1;%NEU_E;
dynamicTargetRequest.ZV = 0;
dynamicTargetRequest.Yaw = 0; % in radian
dynamicTargetRequest.YawRate = -atan2(1,5);
dynamicTargetRequest.Bitmask = bin2dec('1111011111000111');%    65479; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
%                                            '1111101111000111'
%waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 5);
disp('Call dynamic target command');
waypointResponse = call(ROS_MACE.kinematicTargetClient, dynamicTargetRequest, 'Timeout', 5);

for k =1:30
    msg = ROS_MACE.positionSub.LatestMessage;
    positionCallback(ROS_MACE,msg);
    pause(1);
end

disp('Press any key to land...')
pause;
land( ROS_MACE );