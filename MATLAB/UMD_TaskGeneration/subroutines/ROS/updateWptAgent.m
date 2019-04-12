function updateWptAgent( ROS_MACE, wptsXY, agentIndex )

% Setup waypoint command
waypointRequest = rosmessage(ROS_MACE.waypointClient);
waypointRequest.Timestamp = rostime('now');
waypointRequest.VehicleID = ROS_MACE.agentIDs(agentIndex); % Vehicle ID
waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
switch ROS_MACE.coordSys
    case 'ENU'
        waypointRequest.Easting = wptsXY(1); % Relative easting position to Datum
        waypointRequest.Northing = wptsXY(2); % Relative northing position to Datum
    case 'F3'
        [east, north] = F3toENU(wptsXY(1), wptsXY(2));
        waypointRequest.Easting = east; % Relative easting position to Datum
        waypointRequest.Northing = north; % Relative northing position to Datum
end
waypointRequest.Altitude = ROS_MACE.operationalAlt(agentIndex);
waypointResponse = call(ROS_MACE.waypointClient, waypointRequest, 'Timeout', 5);
if ( waypointResponse.Success )
    fprintf('VehicleID %d Waypoint Command Sent = (%3.1f, %3.1f).\n',waypointRequest.VehicleID,wptsXY(1),wptsXY(2));
else
    fprintf('VehicleID %d Waypoint Command Failed.\n',waypointRequest.VehicleID);
end


end