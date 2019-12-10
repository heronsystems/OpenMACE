function bundleManager( ROS_MACE, wptsDesired, captureRadius)

global agentTargetHist tStart

reaching_wpts_end = zeros(1,ROS_MACE.N);
cur_wptIndex = ones(1,ROS_MACE.N);

colors=['rbmckgy'];

for agentIndex = 1:ROS_MACE.N
    waypointRequest = rosmessage(ROS_MACE.waypointClient);
    waypointRequest.Timestamp = rostime('now');
    waypointRequest.VehicleID = ROS_MACE.agentIDs(agentIndex);   % Vehicle ID
    waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
    
    targetWpt = wptsDesired{agentIndex}(1,:);
    switch ROS_MACE.coordSys
        case 'ENU'
            waypointRequest.Easting = targetWpt(1); % Relative easting position to Datum
            waypointRequest.Northing = targetWpt(2); % Relative northing position to Datum
        case 'F3'
            [east, north] = F3toENU(targetWpt(1),targetWpt(2));
            waypointRequest.Easting = east; % Relative easting position to Datum
            waypointRequest.Northing = north; % Relative northing position to Datum
    end
    waypointRequest.Altitude = ROS_MACE.operationalAlt(agentIndex);
    waypointResponse = call(ROS_MACE.waypointClient, waypointRequest, 'Timeout', 10);
    
    if ( waypointResponse.Success )
        % record the time of successful wpt request
        agentTargetHist{agentIndex} = [agentTargetHist{agentIndex} [toc(tStart); targetWpt(1); targetWpt(2);waypointRequest.Altitude]];
        fprintf('VehicleID %d Waypoint Command Sent = (%3.1f, %3.1f).\n',ROS_MACE.agentIDs(agentIndex),targetWpt(1),targetWpt(2));
        
        subplot(ROS_MACE.taskAndLocation);
        plot(wptsDesired{agentIndex}(:,1),wptsDesired{agentIndex}(:,2),[colors(agentIndex+2) '-']);
        plot(targetWpt(1), targetWpt(2),[colors(agentIndex) '+'],'MarkerSize',4,'linewidth',2);
        drawnow;
    else
        fprintf('VehicleID %d Waypoint Command Failed.\n',ROS_MACE.agentIDs(agentIndex));
    end
end



while(~all(reaching_wpts_end))
    % update location ======
    msg = ROS_MACE.positionSub.LatestMessage;
    % it appears that the LatestMessage circumvents the callback, so we
    % force it here:
    positionCallback( ROS_MACE, msg );
    % check distance =====
    if ( ~isempty(msg) )
        agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
        switch ROS_MACE.coordSys
            case 'ENU'
                pos = [msg.Easting msg.Northing];
            case 'F3'
                [xF3, yF3] = ENUtoF3( msg.Easting , msg.Northing );
                pos = [xF3 yF3];
        end
        dist = norm(pos-[wptsDesired{agentIndex}(cur_wptIndex(agentIndex),1) wptsDesired{agentIndex}(cur_wptIndex(agentIndex),2)]);
        if ( dist <= captureRadius)
            fprintf('VehicleID %d is on station. (+/- %3.1f m).\n', msg.VehicleID, captureRadius);
            
            if cur_wptIndex(agentIndex) < size(wptsDesired{agentIndex},1) % the desired wpt hasn't reached the end
                cur_wptIndex(agentIndex) = cur_wptIndex(agentIndex)+1;
                
                waypointRequest = rosmessage(ROS_MACE.waypointClient);
                waypointRequest.Timestamp = rostime('now');
                waypointRequest.VehicleID = ROS_MACE.agentIDs(agentIndex);   % Vehicle ID
                waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
                
                targetWpt = wptsDesired{agentIndex}(cur_wptIndex(agentIndex),:);
                switch ROS_MACE.coordSys
                    case 'ENU'
                        waypointRequest.Easting = targetWpt(1); % Relative easting position to Datum
                        waypointRequest.Northing = targetWpt(2); % Relative northing position to Datum
                    case 'F3'
                        [east, north] = F3toENU(targetWpt(1),targetWpt(2));
                        waypointRequest.Easting = east; % Relative easting position to Datum
                        waypointRequest.Northing = north; % Relative northing position to Datum
                end
                waypointRequest.Altitude = ROS_MACE.operationalAlt(agentIndex);
                waypointResponse = call(ROS_MACE.waypointClient, waypointRequest, 'Timeout', 10);
                
                if ( waypointResponse.Success )
                    % record the time of successful wpt request
                    agentTargetHist{agentIndex} = [agentTargetHist{agentIndex} [toc(tStart); targetWpt(1); targetWpt(2);waypointRequest.Altitude]];
                    fprintf('VehicleID %d Waypoint Command Sent = (%3.1f, %3.1f).\n',ROS_MACE.agentIDs(agentIndex),targetWpt(1),targetWpt(2));
                    
                    subplot(ROS_MACE.taskAndLocation);
                    plot(targetWpt(1), targetWpt(2),[colors(agentIndex) '+'],'MarkerSize',4,'linewidth',2);
                    drawnow;
                else
                    fprintf('VehicleID %d Waypoint Command Failed.\n',ROS_MACE.agentIDs(agentIndex));
                end
                %     pause(0.25);
            else
                reaching_wpts_end(agentIndex) = 1;
            end
        else
%             fprintf('VehicleID %d is %3.1f m (< %3.1f m) away from station.\n', msg.VehicleID, dist, captureRadius);
        end
        
    end
    %     pause(0.1);
end

end