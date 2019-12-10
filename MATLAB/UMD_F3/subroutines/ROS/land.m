function land( ROS_MACE )

global agentPosition

for i = 1:1:ROS_MACE.N
    % Setup Land command:
    landRequest = rosmessage(ROS_MACE.landClient);
    landRequest.Timestamp = rostime('now');
    landRequest.VehicleID = ROS_MACE.agentIDs(i);  
    landRequest.CommandID = 4; % TODO: Set command ID enum in MACE
    landResponse = call(ROS_MACE.landClient, landRequest, 'Timeout', 5);
    if ( landResponse.Success )
        fprintf('VehicleID %d Land Command Sent.\n',i);
    else 
        fprintf('VehicleID %d Land Command Failed.\n',i);
    end  
end

disp('Waiting for landing to complete...')
% Wait for each vehicle to achieve takeoff altitude
landingAchieved = zeros(1,ROS_MACE.N);

while( ~all(landingAchieved) )
    for k = 1:ROS_MACE.N
        if (abs(agentPosition(k,3)) <= 0.5) 
            landingAchieved(k) = 1;
            fprintf('VehicleID %d Reached Ground  (+/- 0.5 m).\n', ROS_MACE.agentIDs(k));
        end
        pause(0.1);
    end
end

% while( ~all(landingAchieved) )
%     msg = ROS_MACE.positionSub.LatestMessage;
% %     msgGeo = ROS_MACE.geopositionSub.LatestMessage;
% %     positionCallback( ROS_MACE, msg); 
%     updatePlot(ROS_MACE);
%     if ( ~isempty(msg) )
%         agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
%         if ( landingAchieved(agentIndex) == 0 )
%             if ( abs(abs(msg.Altitude) ) <= 0.75 )
%                 landingAchieved(agentIndex) = 1;
%                 fprintf('VehicleID %d Reached Ground  (+/- 0.5 m).\n', msg.VehicleID);
%             end
%         end
%     end
%     pause(0.1);
% end

disp('Run Complete.')
% killMACE;
% killMACE;
rosshutdown;
end