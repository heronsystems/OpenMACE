function updateWpts( ROS_MACE, wptsXY ) 
for i = 1:1:ROS_MACE.N 
    % Setup waypoint command
    waypointRequest = rosmessage(ROS_MACE.waypointClient);
    waypointRequest.Timestamp = rostime('now');
    waypointRequest.VehicleID = ROS_MACE.agentIDs(i);   % Vehicle ID
    waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
    switch ROS_MACE.coordSys
        case 'ENU'
            waypointRequest.Easting = wptsXY(i,1); % Relative easting position to Datum
            waypointRequest.Northing = wptsXY(i,2); % Relative northing position to Datum            
        case 'F3'
            [east, north] = F3toENU(wptsXY(i,1), wptsXY(i,2));
            waypointRequest.Easting = east; % Relative easting position to Datum 
            waypointRequest.Northing = north; % Relative northing position to Datum                                
   end               
    waypointRequest.Altitude = ROS_MACE.operationalAlt(i);
    waypointResponse = call(ROS_MACE.waypointClient, waypointRequest, 'Timeout', 10);
       
    if ( waypointResponse.Success )
%         % record the time of successful wpt request
%         % uncomment the following two lines during checkout flight
%         global agentTargetHist tStart
%         agentTargetHist{i} = [agentTargetHist{i} [toc(tStart); wptsXY(i,1); wptsXY(i,2);waypointRequest.Altitude]];
        fprintf('VehicleID %d Waypoint Command Sent = (%3.1f, %3.1f).\n',ROS_MACE.agentIDs(i),wptsXY(i,1),wptsXY(i,2));
    else 
        fprintf('VehicleID %d Waypoint Command Failed.\n',ROS_MACE.agentIDs(i));
    end    
%     pause(0.25);
end

end