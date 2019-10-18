function dispatchSwarmToPerimeter( ROS_MACE , trueWorld )
% initialize agent states
% distribute the agents uniformly along the boundary,
% prevPt indicates a node on the (xpoly,ypoly) that is closest to the agent
[xv, yv, ~] = distributeUniformlyAlongCurve(ROS_MACE.N,trueWorld.xpoly,trueWorld.ypoly);

disp('Press a key to dispatch vehicles to search area...')
pause;
countdownVerbose(3);

for i = 1:1:ROS_MACE.N
    % Setup waypoint command
    waypointRequest = rosmessage(ROS_MACE.waypointClient);
    waypointRequest.Timestamp = rostime('now');
    waypointRequest.VehicleID = ROS_MACE.agentIDs(i); % Vehicle ID
    waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
    switch ROS_MACE.coordSys
        case 'ENU'
            waypointRequest.Easting = xv(i); % Relative easting position to Datum
            waypointRequest.Northing = yv(i); % Relative northing position to Datum
        case 'F3'
            [east, north] = F3toENU( xv(i) , yv(i) );
            waypointRequest.Easting = east; % Relative easting position to Datum
            waypointRequest.Northing = north; % Relative northing position to Datum
    end
    waypointRequest.Altitude = ROS_MACE.operationalAlt(i);
    waypointResponse = call(ROS_MACE.waypointClient, waypointRequest, 'Timeout', 5);
    if ( waypointResponse.Success )
        fprintf('VehicleID %d Waypoint Command Sent.\n',i);
    else
        fprintf('VehicleID %d Waypoint Command Failed.\n',i);
    end
end
%
disp('Waiting for vehicles to reach station...')
% Wait for each vehicle to achieve takeoff altitude
stationAchieved = zeros(1,ROS_MACE.N);
k = 1;
while( ~all(stationAchieved) )
    msg = ROS_MACE.positionSub.LatestMessage;
    positionCallback( ROS_MACE, msg );
    if ( ~isempty(msg) )
        agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
        if ( stationAchieved(agentIndex) == 0 )
            switch ROS_MACE.coordSys
                case 'ENU'
                    pos = [msg.Easting msg.Northing];
                case 'F3'
%                     figure(100);
                    % debugging
%                     i = 1;
%                     if (msg.VehicleID == i)
%                         [east, north] = F3toENU( xv(i) , yv(i) );
%                         plot(k,east,'ro');
%                         hold on;
%                         plot(k,msg.Easting,'r+');
%                         plot(k,north,'bo');
%                         plot(k,msg.Northing,'b+');
%                         k = k + 1;
%                     end
                    %
                    [xF3, yF3] = ENUtoF3( msg.Easting , msg.Northing );
                    pos = [xF3 yF3];
            end
            dist = norm(pos-[xv(agentIndex) yv(agentIndex)]);
            if ( dist <= 2.00 )
                stationAchieved(agentIndex) = 1;
                fprintf('VehicleID %d is on station. (+/- 2.00 m).\n', msg.VehicleID);
            end
        end
    end
end



end
