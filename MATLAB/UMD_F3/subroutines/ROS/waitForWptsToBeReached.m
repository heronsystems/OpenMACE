function waitForWptsToBeReached( ROS_MACE, wpts, captureRadius )

stationAchieved = zeros(1,ROS_MACE.N);
k = 1;
while( ~all(stationAchieved) )
    msg = ROS_MACE.positionSub.LatestMessage;
%     msgGeo = ROS_MACE.geopositionSub.LatestMessage;  
%     positionCallback( ROS_MACE, msg);
    updatePlot(ROS_MACE);
    if ( ~isempty(msg))    
        agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
        if ( stationAchieved(agentIndex) == 0 )
            switch ROS_MACE.coordSys
                case 'ENU'
%                     [Easting, Northing,~] = geodetic2enu(msgGeo.Latitude,msgGeo.Longitude,0,ROS_MACE.LatRef,ROS_MACE.LongRef,0,wgs84Ellipsoid,'degrees');
                    pos = [msg.Easting msg.Northing];
                case 'F3'
%                     [Easting, Northing,~] = geodetic2enu(msgGeo.Latitude,msgGeo.Longitude,0,ROS_MACE.LatRef,ROS_MACE.LongRef,0,wgs84Ellipsoid,'degrees');
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