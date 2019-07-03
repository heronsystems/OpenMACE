function [ outputArgument ] = positionCallback( ROS_MACE, msg)
%   ROS UPDATE_POSITION message with properties:
% 
%     MessageType: 'mace_matlab/UPDATE_POSITION'
%       Timestamp: [1×1 Time]
%       VehicleID: 0
%        Northing: 0
%         Easting: 0
%        Altitude: 0
%      NorthSpeed: 0
%       EastSpeed: 0

    global tStart;
    colors=['rbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcy'];
    time = toc(tStart);
    if ( ~isempty(msg) )   
    subplot(ROS_MACE.altitude);
    plot(time,msg.Altitude,[colors(msg.VehicleID) 'o']);
    hold on;
    if ( time > 30 )
        xlim([time-30 time])
    end
    drawnow;
    
    % plot position
    [xf3, yf3] = ENUtoF3(msg.Easting, msg.Northing);
    subplot(ROS_MACE.taskAndLocation);
    plot(xf3,yf3,[colors(msg.VehicleID) 'o']);
    hold on;
    
    drawnow;
       
    % store the new location along with time
    % uncomment the following line for checkout flight (Sheng)
%     global agentStateHist;
%     agentStateHist{msg.VehicleID} = [agentStateHist{msg.VehicleID} [time;xf3;yf3;msg.Altitude]];
    end
end
    

