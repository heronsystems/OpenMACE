function waitForWptsToBeReached( ROS_MACE, wpts, captureRadius )

stationAchieved = zeros(1,ROS_MACE.N);
k = 1;
while( ~all(stationAchieved) )
%     msg = ROS_MACE.positionSub.LatestMessage;
    msgGeo = ROS_MACE.geopositionSub.LatestMessage;
    % it appears that the LatestMessage circumvents the callback, so we
    % force it here:   
%     positionCallback( ROS_MACE, msg, msgGeo ); 
    positionCallback( ROS_MACE, msgGeo); 
%     if ( ~isempty(msg) && ~isempty(msgGeo) )
    if ( ~isempty(msgGeo))    
        agentIndex = ROS_MACE.agentIDtoIndex( msgGeo.VehicleID );
        if ( stationAchieved(agentIndex) == 0 )
            switch ROS_MACE.coordSys
                case 'ENU'
                    [Easting, Northing,~] = geodetic2enu(msgGeo.Latitude,msgGeo.Longitude,0,ROS_MACE.LatRef,ROS_MACE.LongRef,0,wgs84Ellipsoid,'degrees');
                    pos = [Easting Northing];
                case 'F3'
                    [Easting, Northing,~] = geodetic2enu(msgGeo.Latitude,msgGeo.Longitude,0,ROS_MACE.LatRef,ROS_MACE.LongRef,0,wgs84Ellipsoid,'degrees');
                    [xF3, yF3] = ENUtoF3( Easting , Northing );
                    pos = [xF3 yF3];
            end
            dist = norm(pos-[wpts(agentIndex,1) wpts(agentIndex,2)]);
            if ( dist <= captureRadius && stationAchieved(agentIndex) ~= 1 )
                stationAchieved(agentIndex) = 1;
                fprintf('VehicleID %d is on station. (+/- %3.1f m).\n', msgGeo.VehicleID, captureRadius);
            elseif ( stationAchieved(agentIndex) ~= 1 )
                fprintf('VehicleID %d is %3.1f m (< %3.1f m) away from station.\n', msgGeo.VehicleID, dist, captureRadius);
            end
        end
    end    
%     pause(0.1);
end

end