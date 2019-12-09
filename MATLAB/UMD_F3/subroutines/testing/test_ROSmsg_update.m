% Test script for alternative, loss-free ros msg receiving scheme.
clear all; close all; clc;
rosshutdown; % in case there is a leftover instance

% Setup ROS_MACE
[runParams, ROS_MACE] = loadParams_cityblocksAtF3();
ROS_MACE = setupF3FlightTestPlot( runParams, ROS_MACE);

% temporary fix to allow plotting with time on ROS message callback
% will be replaced with MACE timestamp when available
global tStart;
tStart = tic;


ROS_MACE.N = 4;
%ROS_MACE.operationalAlt = [4 8]; % m
%ROS_MACE.agentIDs = [1 2]; % m
ROS_MACE.operationalAlt = [4 5 2 3]; % m
ROS_MACE.agentIDs = [1 2 3 4]; % SYSID_THISMAV on each quadrotor
% warning: only support one quad mission

ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
ROS_MACE.wptCoordinator = 'integrated';

for i = 1:1:length(ROS_MACE.agentIDs)
    ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
end

global agentAngle 
agentAngle = nan(ROS_MACE.N,1);



% ------------------------Taken from launchROS.m ------------------------
rosshutdown;
%robotics.ros.Core;
setenv('ROS_MASTER_URI',['http://' ROS_MACE.ip ':11311']) % ROS Core location
setenv('ROS_IP',ROS_MACE.ip) % MATLAB location
disp('Waiting for a rosmaster...')
rosStarted = 0;
while( rosStarted == 0 )
    try
        rosinit;
        rosStarted = 1;
        disp('Rosmaster found.')
    catch
        disp('Waiting for a rosmaster...')
        rosStarted = 0;
    end
    pause(3);
end

% set rate of loop
% r = robotics.Rate(30); % shall we make it to 5?
% reset(r);

% List ROS topics:
disp('Waiting for MACE topics...')
topiclist = rostopic('list');
MACEreadyFlag = 0;
while ( MACEreadyFlag == 0 )
    for i = 1:1:length(topiclist)
        % check list
        if ( ~isempty(strfind(topiclist{i}, 'MACE')) )
            MACEreadyFlag = 1;
        end
        % update topic list
        topiclist = rostopic('list');
    end
end
disp('Found MACE topics.')
% Set up subscribers:
ROS_MACE.positionSub = rossubscriber('/MACE/UPDATE_LOCAL_POSITION','BufferSize', 10);% @positionCallback, 'BufferSize', 10);
ROS_MACE.geopositionSub = rossubscriber('/MACE/UPDATE_GEODETIC_POSITION','BufferSize', 10);% @positionCallback, 'BufferSize', 10);
ROS_MACE.attitudeSub = rossubscriber('/MACE/UPDATE_ATTITUDE', 'BufferSize', 10);
ROS_MACE.attitudeSub.NewMessageFcn = {@test_angleCallback,ROS_MACE.agentIDtoIndex};
%batterySub = rossubscriber('/MACE/UPDATE_BATTERY', @batteryCallback, 'BufferSize', 10);
%gpsSub = rossubscriber('/MACE/UPDATE_GPS', @gpsCallback, 'BufferSize', 10);
%heartbeatSub = rossubscriber('/MACE/UPDATE_HEARTBEAT', @heartbeatCallback, 'BufferSize', 10);
%vehicleTargetSub = rossubscriber('/MACE/TARGET_STATUS',@vehicleTargetCallback, 'BufferSize', 10);


% Set up service clients:
ROS_MACE.armClient = rossvcclient('command_arm');
ROS_MACE.takeoffClient = rossvcclient('command_takeoff');
ROS_MACE.landClient = rossvcclient('command_land');
ROS_MACE.datumClient = rossvcclient('command_datum');
ROS_MACE.waypointClient = rossvcclient('command_waypoint');
ROS_MACE.kinematicTargetClient = rossvcclient('command_dynamic_target_kinematic');

% if using standalone wptCoordinator, then a service and a callback
% function must be defined
if strcmp(ROS_MACE.wptCoordinator,'standalone')
    ROS_MACE.bundleServer = rossvcserver('/bundle_server','bundle_manager/BUNDLE_REQUEST',@ROSBundleServer);
    servicelist = rosservice('list');
    bundleServerReadyFlag = 0;
    while ( bundleServerReadyFlag == 0 )
        for i = 1:1:length(servicelist)
            % check list
            if ( ~isempty(strfind(servicelist{i}, 'bundle_server')) )
                bundleServerReadyFlag = 1;
            end
            servicelist = rosservice('list');
        end
    end

    fprintf('/bundle_server initialized \n');
end

% --------------------------- end LaunchROS.m --------------------

land(ROS_MACE);