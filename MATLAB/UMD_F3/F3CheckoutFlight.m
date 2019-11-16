% Main script for waypoint and speed+heading controlled flights at F3 using 
% MATLAB
% This script is adopted from main_F3CheckoutFlight.m by A. Wolek and S.
% Cheng, Sept. 2018 - Sept. 2019
% 
% S. Cheng, Oct. 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;
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


% ============= Test 1: N Quads Takeoff, Wpt Mission, and Land ==============
ROS_MACE.N = 1;
%ROS_MACE.operationalAlt = [4 8]; % m
%ROS_MACE.agentIDs = [1 2]; % m
ROS_MACE.operationalAlt = [2]; % m
ROS_MACE.agentIDs = [5]; % SYSID_THISMAV on each quadrotor

ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
ROS_MACE.wptCoordinator = 'integrated';

for i = 1:1:length(ROS_MACE.agentIDs)
    ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
end

ROS_MACE = launchROS( ROS_MACE );
swarmState = sendDatumAndWaitForGPS( ROS_MACE );
armAndTakeoff( ROS_MACE );
disp('Press any key to launch waypoint mission...')
pause;

% evenly distribute N quads between y from 1m to 11m and -11m to -1m
temp = linspace(0,20,ROS_MACE.N+2);
temp = temp(2:end-1);
yLocation = temp(temp<10)-11;
yLocation = [yLocation temp(temp>=10)-9];

wpts = cell(1,ROS_MACE.N);
% 
% for k = 1:ROS_MACE.N
%     if yLocation(k)>0
%         wpts{k} = [11-yLocation(k) yLocation(k);...
%                    1-yLocation(k) yLocation(k);...
%                    11-yLocation(k) yLocation(k)];
%     else
%         wpts{k} = [11+yLocation(k) yLocation(k);...
%                    1+yLocation(k) yLocation(k);...
%                    11+yLocation(k) yLocation(k)];
%     end
% end
% wpts{1} = [5 6;-15 6;10 6;]; % each vector is for a single agent
wpts{1} = [5 -6;-15 -6;10 -6];


captureRadius = 1;% 1.2;
wptManager( ROS_MACE, wpts, captureRadius);

disp('Press any key to land...')
pause;
land( ROS_MACE );

% % ============= Test 2: 1 Quad Takeoff, ascending waypoints, and Land ==============
% ROS_MACE.N = 1;
% %ROS_MACE.operationalAlt = [4 8]; % m
% %ROS_MACE.agentIDs = [1 2]; % m
% ROS_MACE.operationalAlt = [2]; % m
% ROS_MACE.agentIDs = [3]; % SYSID_THISMAV on each quadrotor
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
% wpts{1} = [10 -5]; % push the quad to some place near the shed
% 
% captureRadius = 1;% 1.2;
% wptManager( ROS_MACE, wpts, captureRadius);
% 
% altitudeSequence = [ROS_MACE.operationalAlt(1) 2.5 3 3.5  4 4.5 4.7 4.9 5 5.1 5.2];
% 
% for k = 1:length(altitudeSequence)
%     waypointRequest = rosmessage(ROS_MACE.waypointClient);
%     waypointRequest.Timestamp = rostime('now');
%     waypointRequest.VehicleID = ROS_MACE.agentIDs(i);   % Vehicle ID
%     waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
%     switch ROS_MACE.coordSys
%         case 'ENU'
%             waypointRequest.Easting = wpts{1}(1,1); % Relative easting position to Datum
%             waypointRequest.Northing = wpts{1}(1,2); % Relative northing position to Datum
%         case 'F3'
%             [east, north] = F3toENU(wpts{1}(1,1), wpts{1}(1,2));
%             waypointRequest.Easting = east; % Relative easting position to Datum
%             waypointRequest.Northing = north; % Relative northing position to Datum
%     end
%     waypointRequest.Altitude = altitudeSequence(k);
%     fprintf('Commanding quad to alt = %f.\n',altitudeSequence(k));
%     waypointResponse = call(ROS_MACE.waypointClient, waypointRequest, 'Timeout', 10);
%     
%     % pausing for 10 seconds (total will be about 220 s)
%     for j = 1:20
%         msg = ROS_MACE.positionSub.LatestMessage;
%         pause(0.5);
%         positionCallback(ROS_MACE,msg);
%     end
% end
% 
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );

% % ============= Test 3: quad takeoff, kinematic command, and land =========
% 
% ROS_MACE.N = 1;
% %ROS_MACE.operationalAlt = [4 8]; % m
% %ROS_MACE.agentIDs = [1 2]; % m
% ROS_MACE.operationalAlt = [2]; % m
% ROS_MACE.agentIDs = [3]; % SYSID_THISMAV on each quadrotor
% % warning: only support one quad mission
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
% disp('Press any key to launch the kinematic mission...')
% pause;
% 
% wpts{1} = [10 -5]; % reference to the waypoint mission that flies closer 
% 
% captureRadius = 1;% 1.2;
% wptManager( ROS_MACE, wpts, captureRadius);
% 
% 
% fprintf('Pointing toward the circle center.\n');
% kinematicLocalCommand(ROS_MACE,ROS_MACE.agentIDs,[],[],[],'ENU',0,0,0,'ENU',pi/2,[]);
% 
% 
% for k =1:4
% %     msgGeo = ROS_MACE.geopositionSub.LatestMessage;
%     msg = ROS_MACE.positionSub.LatestMessage;
%     positionCallback(ROS_MACE,msg);
%     pause(1);
% end
% 
% fprintf('start spinning.\n');
% kinematicLocalCommand(ROS_MACE,ROS_MACE.agentIDs,[],[],[],'ENU',1,0,0,'RFU',[],atan2(1,5));
% 
% 
% for k =1:30
% %     msgGeo = ROS_MACE.geopositionSub.LatestMessage;
%     msg = ROS_MACE.positionSub.LatestMessage;
%     positionCallback(ROS_MACE,msg);
%     pause(1);
% end
% 
% fprintf('stop.\n');
% kinematicLocalCommand(ROS_MACE,ROS_MACE.agentIDs,[],[],[],'ENU',0,0,0,'ENU',[],0);
% 
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );

% % ============= Test 4: 1 quad takeoff, follow spline trajectory by kinematic command, and land =========
% 
% ROS_MACE.N = 1;
% %ROS_MACE.operationalAlt = [4 8]; % m
% %ROS_MACE.agentIDs = [1 2]; % m
% ROS_MACE.operationalAlt = [3]; % m
% ROS_MACE.agentIDs = [1]; % SYSID_THISMAV on each quadrotor
% % warning: only support one quad mission
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
% disp('Press any key to launch the kinematic mission...')
% pause;
% 
% wpts{1} = [5 4]; % reference to the waypoint mission that flies closer 
% 
% captureRadius = 1;% 1.2;
% % wptManager(ROS_MACE, wpts, captureRadius);
% 
% 
% % ========= splines ================
% commandingTime = 0.5;
% 
% % define the speed function
% v = @(t) 1;
% 
% % circular trajectory
% tpx = [5 8 5 -2 -10 -13 -10 -2 5];
% tpy = [5 0 -5  -6 -5 0 5  6 5];
% 
% % snake trajectory
% % tpx = [12 9 6 3 0 -3 -6 -9 -12 -12 12 12 12];
% % tpy = [-4 -8 -4 -8 -4 -8 -4 -8 -4 4 4 0 -4];
% 
% % subplot(1,2,1);
% % plot(tpx,tpy,tpx,tpy,'o');axis equal;
% 
% % anchor times
% % t = linspace(0,60,length(tpx));
% % query times
% % t_query = 0:1:max(t);
% 
% % generate consistant trajectory as the function "distance2curve_modified" does
% seglen = sqrt(sum(diff([tpx;tpy]',[],1).^2,2));
% t = [0;cumsum(seglen)/sum(seglen)];
% 
% % x_query = spline(t,[tpx],t_query);
% % y_query = spline(t,[tpy],t_query);
% 
% x_query = spline(t,[tpx],0:0.01:1);
% y_query = spline(t,[tpy],0:0.01:1);
% 
% 
% % x_coef = spline(t,[tpx]);
% % y_coef = spline(t,[tpy]);
% 
% % plot the reference trajectory
% plot(ROS_MACE.taskAndLocation, tpx,tpy,'s',x_query,y_query,':.');
% 
% % start a new figure for the test
% figure(2);
% ROS_MACE.traj = axes;
% plot(ROS_MACE.traj,tpx,tpy,'s',x_query,y_query,':.');axis equal;hold on;
% axis([-15,10,-8,8]);
% 
% px = wpts{1}(1);
% py = wpts{1}(2);
% 
% v_tan_stor = [];
% v_norm_stor = [];
% 
% for k = 1:100
%     msg = ROS_MACE.positionSub.LatestMessage;
%     positionCallback(ROS_MACE,msg);
%     
%     [xF3, yF3] = ENUtoF3( msg.Easting , msg.Northing );
%     
%     % compute the closet point on the spline, and its distance and fractional
%     % arc length
%     [xy,distance,t_a,coef,t_b] = distance2curve_modified([tpx;tpy]',[xF3,yF3],'spline');
%     
%     px = [px xF3];
%     py = [py yF3];
%     
%     % x0 and y0 are the closed point to the vehicle current location on the desired trajectory
%     x0 = xy(1);
%     y0 = xy(2);
%     
%     % dist is the minimum distance
%     dist = distance;
%     
%     vx = 0;
%     vy = 0;
%     
%     for j = 1:size(coef,2)-1
%         vx = vx + coef(1,end-j)*j*(t_a-t_b)^j;
%         vy = vy + coef(2,end-j)*j*(t_a-t_b)^j;
%     end
%     
%     v_tan = [vx vy]/norm([vx vy])*v(k*commandingTime);
%     v_norm = 0.2*[x0-px(end) y0-py(end)]/commandingTime;
%     
%     v_tan_stor = [v_tan_stor;v_tan];
%     v_norm_stor = [v_norm_stor;v_norm];
%     
%     
%     kinematicLocalCommand(ROS_MACE,ROS_MACE.agentIDs,[],[],[],'ENU',v_tan(1)+v_norm(1),v_tan(2)+v_norm(2),0,'ENU',[],[]);
%     
%     fprintf('Distance to given trajectory is %.2f m.\nNormal velocity is %.2f m/s.\n',dist,norm(v_norm));
%     
%     h_agent = plot(ROS_MACE.traj,px(end),py(end),'ro');
%     h_agent_vtan = quiver(ROS_MACE.traj,px(end),py(end),v_tan(1),v_tan(2),'Color','r');
%     h_agent_vnorm = quiver(ROS_MACE.traj,px(end),py(end),v_norm(1),v_norm(2),'Color','b');
%     
%     drawnow;
%     
%     pause(commandingTime);
%     set(h_agent,'Color','k');
%     set(h_agent_vtan,'visible','off');
%     set(h_agent_vnorm,'visible','off');
% end
% 
% fprintf('Stop the vehicle.\n');
% kinematicLocalCommand(ROS_MACE,ROS_MACE.agentIDs,[],[],[],'ENU',0,0,0,'ENU',[],[]);
% 
% disp('Press any key to land...')
% pause;
% land( ROS_MACE );