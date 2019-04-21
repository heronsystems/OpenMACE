% Main script for checkout flights at F3 using MATLAB
% A. Wolek, S. Cheng, Dec. 2018
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
[runParams, ROS_MACE] = loadParams_testingF3();
ROS_MACE = setupF3FlightTestPlot( runParams, ROS_MACE);

% temporary fix to allow plotting with time on ROS message callback
% will be replaced with MACE timestamp when available
global tStart;
tStart = tic;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test: Single Quad Takeoff and Land
% 
% ROS_MACE.N = 1;
% ROS_MACE.operationalAlt = [5]; % m
% ROS_MACE.agentIDs = [1]; % m
% ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
% for i = 1:1:length(ROS_MACE.agentIDs)
%     ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
% end
% 
% ROS_MACE = launchROS( ROS_MACE );
% swarmState = sendDatumAndWaitForGPS( ROS_MACE );
% armAndTakeoff( ROS_MACE );
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test: Two Quads Takeoff and Land
% ROS_MACE.N = 2;
% ROS_MACE = launchROS( ROS_MACE );
% swarmState = sendDatumAndWaitForGPS( ROS_MACE );
% armAndTakeoff( ROS_MACE );
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Test: One Quad Takeoff, Wpt Mission, and Land
% ROS_MACE.N = 1;
% ROS_MACE.operationalAlt = [5]; % m
% ROS_MACE.agentIDs = [1]; % m
% ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
% for i = 1:1:length(ROS_MACE.agentIDs)
%     ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
% end
% ROS_MACE = launchROS( ROS_MACE );
% swarmState = sendDatumAndWaitForGPS( ROS_MACE );
% armAndTakeoff( ROS_MACE );
% disp('Press any key to launch waypoint mission...')
% pause;
% wpts{1} = [5 6;-15 6;8 6]; % each vector is for a single agent
% captureRadius = 0.75;
% wptManager( ROS_MACE, wpts, captureRadius);
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Test: Two Quad Takeoff, Wpt Mission, and Land
% 
% ROS_MACE.N = 2;
% ROS_MACE.operationalAlt = [4 8]; % m
% ROS_MACE.agentIDs = [1 2]; % m
% ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
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
% wpts{1} = [5 6;-15 6;10 6;]; % each vector is for a single agent
% wpts{2} = [5 -6;-15 -6;10 -6];
% 
% 
% captureRadius = 1;% 1.2;
% wptManager( ROS_MACE, wpts, captureRadius);
% 
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test: Two Quad Takeoff, bundle of gradual varying length and angle waypoinys, and Land

ROS_MACE.N = 2;
ROS_MACE.operationalAlt = [4 8]; % m
ROS_MACE.agentIDs = [1 2]; % m
ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));

global agentStateHist
global agentTargetHist

for i = 1:1:length(ROS_MACE.agentIDs)
    ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
    agentStateHist{i} = [];
    agentTargetHist{i} = [];
end

ROS_MACE = launchROS( ROS_MACE );
swarmState = sendDatumAndWaitForGPS( ROS_MACE );
armAndTakeoff( ROS_MACE );
disp('Press any key to launch waypoint mission...')
pause;

% two circles
% wpts{1} = [10+3*cos(0:pi/4:2*pi);4+3*sin(0:pi/4:2*pi)]';
% wpts{2} = [10-3*cos(0:pi/4:2*pi);-4-3*sin(0:pi/4:2*pi)]';

% random waypoints
% m = 30;
% for i = 1:ROS_MACE.N
%     wpts{i} = 3*ones(m,2);
%     for k = 1:m % generate random waypoints that resemble a bundle (within 1/3 of F3)
%         if k == 1
%             wpts{i}(k,:) = [14+2*(rand-0.5) (-1)^i*7+rand-0.5];
%         else
%             while(1)
%                 saverand1 = 2*(rand-0.5)*pi/2+pi; % angle ranges from -90 to 90 deg with a 180 deg bias
%                 saverand2 = 1.5*(rand+1); % distance ranges from 1.5 to 3 m
%                 wpts{i}(k,:) = wpts{i}(k-1,:)+[saverand2*cos(saverand1) saverand2*sin(saverand1)];
%                 if ~( wpts{i}(k,1)>=26.5 || wpts{i}(k,1)<=-57.5 || wpts{i}(k,2)>=11.5 || wpts{i}(k,2)<=-11.5)
%                     break;
%                 end
%             end
%         end
%     end
% end

% gradual distance and angle trajectory
wpts{1} = [[5 -7];zeros(39,2)];
wpts{2} = [[5 7];zeros(39,2)];
for i = 1:ROS_MACE.N
    % towards negative x direction
    for k = 2:20
        wpts{i}(k,1) = [ wpts{i}(k-1,1)+(1+k*0.1)*cos(pi + (-1)^k*4*k/90*pi/2)];
        wpts{i}(k,2) = [ wpts{i}(k-1,2)+(1+k*0.1)*sin(pi + (-1)^k*4*k/90*pi/2)];
    end
    % towards positive x direction
    for k = 21:40
        wpts{i}(k,1) = [wpts{i}(k-1,1)+(1+(21-(k-20))*0.1)*cos((-1)^(k-20)*4*(k-20)/90*pi/2)];
        wpts{i}(k,2) = [wpts{i}(k-1,2)+(1+(21-(k-20))*0.1)*sin((-1)^(k-20)*4*(k-20)/90*pi/2)];
    end
end

captureRadius = 1;% 1.2;
% wptManager( ROS_MACE, wpts, captureRadius);
bundleManager( ROS_MACE, wpts, captureRadius);
disp('Press any key to land...')
pause;
land( ROS_MACE );

matFileName = ['F3CheckoutFlight_' datestr(now,'dd_mmm_yyyy_HHMMSS') '.mat']
save(matFileName,'-v7.3');

%
% % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % Test: Two Quad Takeoff, 'Follow the Leader' Wpt Mission, and Land
% % % Test: Two Quad Takeoff, Wpt Mission, and Land
% ROS_MACE.N = 2;
% ROS_MACE.operationalAlt = [4 8]; % m
% ROS_MACE.agentIDs = [1 2]; % m
% leaderID = 1;
% ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
% for i = 1:1:length(ROS_MACE.agentIDs)
%     ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
% end
% ROS_MACE = launchROS( ROS_MACE );
% swarmState = sendDatumAndWaitForGPS( ROS_MACE );
% armAndTakeoff( ROS_MACE );
% disp('Press any key to launch waypoint mission...')
% pause;
% wpts{1} = [5 6]; % each vector is for a single agent
% wpts{2} = [5 -6];
% captureRadius = 1.2;
% wptManager( ROS_MACE, wpts, captureRadius);
% disp('Quads are in position. Press key to begin mission...')
% pause;
% % second quad follows the x values of the leader along a parallel line that
% % is offset and constrained
% % Assume leader is Vehicle ID 1
% leaderWpts = [-15 6;8 6];
% % Assume follower is Vehicle ID 2
% followerMinX = -16;
% followerMaxX = 9;
% followerY = -6;
% 
% followTheLeaderParallelLines(ROS_MACE, leaderWpts, leaderID, captureRadius, followerMinX, followerMaxX, followerY)
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );
