function checkIfWptsReached( ROS_MACE, wpts, captureRadius )

stationAchieved = zeros(1,ROS_MACE.N);
k = 1;
while( ~all(stationAchieved) )
    msg = ROS_MACE.positionSub.LatestMessage;
    if ( ~isempty(msg) )
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
            if ( dist <= captureRadius )
                stationAchieved(agentIndex) = 1;
                fprintf('VehicleID %d is on station. (+/- %3.1f m).\n', msg.VehicleID, captureRadius);
            end
        end
    end
end

end