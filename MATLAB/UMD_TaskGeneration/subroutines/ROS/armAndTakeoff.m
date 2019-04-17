function armAndTakeoff( ROS_MACE )
% Setup Arm vehicle command:
armRequest = rosmessage(ROS_MACE.armClient);
armRequest.CommandID = 1; % TODO: Set command ID enum in MACE
armRequest.ArmCmd = true; % True to ARM throttle, False to DISARM

for i = 1:1:ROS_MACE.N
    armResponse = false;    
    armRequest.Timestamp = rostime('now');    
    armRequest.VehicleID = ROS_MACE.agentIDs(i); % Vehicle ID
    armResponse = call(ROS_MACE.armClient, armRequest, 'Timeout', 5);
    if ( armResponse.Success )
        fprintf('VehicleID %d Arm Command Sent.\n',i);
    else 
        fprintf('VehicleID %d Arm Command Failed.\n',i);
    end    
end
disp('Arm Complete. Begin Takeoff.')
countdownVerbose(3);


% Setup Vehicle takeoff command:
takeoffRequest = rosmessage(ROS_MACE.takeoffClient);
takeoffRequest.CommandID = 2; % TODO: Set command ID enum in MACE
for i = 1:1:ROS_MACE.N    
    takeoffResponse = false;
    takeoffRequest.Timestamp = rostime('now');
    takeoffRequest.VehicleID = ROS_MACE.agentIDs(i);  
    takeoffRequest.TakeoffAlt = ROS_MACE.operationalAlt(i); % Takeoff altitude
    % If you don't set lat/lon (or set them to 0.0), it will takeoff in current position
    % takeoffRequest.LatitudeDeg = 0.0; % If 0.0, takeoff where you currently are
    % takeoffRequest.LongitudeDeg = 0.0; % If 0.0, takeoff where you currently are
    takeoffResponse = call(ROS_MACE.takeoffClient, takeoffRequest, 'Timeout', 5);
    if ( takeoffResponse.Success )
        fprintf('VehicleID %d Takeoff Command Sent.\n',i);
    else 
        fprintf('VehicleID %d Takeoff Command Failed.\n',i);
    end    
end
disp('Waiting for takeoff to complete...')
% Wait for each vehicle to achieve takeoff altitude
takeoffAchieved = zeros(1,ROS_MACE.N);
colors = ['rbk'];

while( ~all(takeoffAchieved) )
    msg = ROS_MACE.positionSub.LatestMessage;   
    positionCallback( ROS_MACE, msg); 
    if ( ~isempty(msg) )
        agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
        if ( takeoffAchieved(agentIndex) == 0 )
            if ( abs(abs(msg.Altitude) - ROS_MACE.operationalAlt(agentIndex)) <= 0.20 )
                takeoffAchieved(agentIndex) = 1;
                fprintf('VehicleID %d Reached Takeoff Altitude (+/- 0.20 m).\n', msg.VehicleID);
            end
        end
    end
    pause(0.1);
end

end