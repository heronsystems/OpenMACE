function [ outputArgument ] = positionCallback( subscriber, msg )
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
    global agentStateHist;
    colors=['rbkmgcy'];
    time = toc(tStart);
    if ( ~isempty(msg) )
    figure(1)
    % plot altitude
    subplot(2,1,1)    
    plot(time,msg.Altitude,[colors(msg.VehicleID) 'o']);
    hold on;
    if ( time > 30 )
        xlim([time-30 time])
    end
    drawnow;
    
    % plot position
    [xf3, yf3] = ENUtoF3(msg.Easting, msg.Northing);
    subplot(2,1,2)  
    plot(xf3,yf3,[colors(msg.VehicleID) 'o']);
    hold on;
    drawnow;
    
    % store the new location along with time
    agentStateHist{msg.VehicleID} = [agentStateHist{msg.VehicleID} [time;xf3;yf3;msg.Altitude]];
    end
end
    

