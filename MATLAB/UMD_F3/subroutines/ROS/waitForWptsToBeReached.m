function waitForWptsToBeReached( ROS_MACE, wpts, captureRadius )

stationAchieved = zeros(1,ROS_MACE.N);
k = 1;
while( ~all(stationAchieved) )
    msg = ROS_MACE.positionSub.LatestMessage;
    msgGeo = ROS_MACE.geopositionSub.LatestMessage;
    % it appears that the LatestMessage circumvents the callback, so we
    % force it here:   
%     positionCallback( ROS_MACE, msg, msgGeo ); 
    positionCallback( ROS_MACE, msg); 
%     if ( ~isempty(msg) && ~isempty(msgGeo) )
    if ( ~isempty(msg))    
        agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
        if ( stationAchieved(agentIndex) == 0 )
            switch ROS_MACE.coordSys
                case 'ENU'
                    pos = [msg.Easting msg.Northing];
                case 'F3'
                    [xF3, yF3] = ENUtoF3( msg.Easting , msg.Northing );
                    pos = [xF3 yF3];
            end
            dist = norm(pos-[wpts(agentIndex,1) wpts(agentIndex,2)]);
            if ( dist <= captureRadius && stationAchieved(agentIndex) ~= 1 )
                stationAchieved(agentIndex) = 1;
                fprintf('VehicleID %d is on station. (+/- %3.1f m).\n', msg.VehicleID, captureRadius);
            elseif ( stationAchieved(agentIndex) ~= 1 )
                fprintf('VehicleID %d is %3.1f m (< %3.1f m) away from station.\n', msg.VehicleID, dist, captureRadius);
            end
        end
    end    
%     pause(0.1);
end

end