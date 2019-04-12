function followTheLeaderParallelLines(ROS_MACE, leaderWpts, leaderID, captureRadius, followerMinX, followerMaxX, followerY)

%
if ( length(ROS_MACE.agentIDs) ~= 2)
   error('Incorrect number of agents')
end
% determine follower ID
leaderIndex = find(ROS_MACE.agentIDs == leaderID);
if ( leaderIndex  == 1 )
    followerIndex = 2;
elseif ( leaderIndex == 2 )
    followerIndex =1;
end
leaderIndex
followerIndex

numWpts = size(leaderWpts,1);

for curWpt = [1:1:numWpts]
    % plot
    figure(1)
    subplot(2,1,2)
    wptsDesired = [ leaderWpts(curWpt,:) ];
    plot(wptsDesired(:,1), wptsDesired(:,2),'k+','MarkerSize',4,'linewidth',2);
    drawnow;
    % send
    updateWptAgent( ROS_MACE, wptsDesired, leaderIndex )
    % wait
    stationAchieved = 0;
    k = 1;
    while( ~stationAchieved )
        msg = ROS_MACE.positionSub.LatestMessage;
        % it appears that the LatestMessage circumvents the callback, so we
        % force it here:
        positionCallback( ROS_MACE.positionSub, msg );
        if ( ~isempty(msg) && (msg.VehicleID == leaderID) )
            switch ROS_MACE.coordSys
                case 'ENU'
                    pos = [msg.Easting msg.Northing];
                case 'F3'
                    [xF3, yF3] = ENUtoF3( msg.Easting , msg.Northing );
                    pos = [xF3 yF3];
            end
            
            % tell follower to go to this x location
            if ( xF3 > followerMaxX )
                wptsFollowerXY(1,1) = followerMaxX;
            elseif ( xF3 < followerMinX )
                wptsFollowerXY(1,1) = followerMinX;
            else
                wptsFollowerXY(1,1) = xF3;
            end
            wptsFollowerXY(1,2) = followerY;
            fprintf('Follower told to go to (x,y) = (%3.1f,%3.1f)\n', wptsFollowerXY(1), wptsFollowerXY(2));
            updateWptAgent( ROS_MACE, wptsFollowerXY, followerIndex )
            
            % check if leader is on station
            if ( stationAchieved == 0 )
                dist = norm(pos-wptsDesired);
                if ( dist <= captureRadius && stationAchieved ~= 1 )
                    stationAchieved = 1;
                    fprintf('Leader: VehicleID %d is on station. (+/- %3.1f m).\n', msg.VehicleID, captureRadius);
                elseif ( stationAchieved ~= 1 )
                    fprintf('Leader: VehicleID %d is %3.1f m (< %3.1f m) away from station.\n', msg.VehicleID, dist, captureRadius);
                end
            end
        end
        pause(0.1);
    end
    disp('**** WPTS ACHIEVED ****');
    disp('Keeping Station for 2 seconds...');
    countdownVerbose(2);
end


end