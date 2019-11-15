function [ outputArgument ] = positionCallback( ROS_MACE, msg)
%   ROS UPDATE_POSITION message with properties:
% 
%     MessageType: 'mace_matlab_msgs/UPDATE_LOCAL_POSITION'
%       Timestamp: [1×1 Time]
%       VehicleID: 0
%           Frame: 0 (newly added, 2019 Oct.)
%        Northing: 0
%         Easting: 0
%        Altitude: 0
%      NorthSpeed: 0
%       EastSpeed: 0


    global tStart;

    colors=['rbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcyrbkmgcy'];
    time = toc(tStart);
    %     if ( ~isempty(msg) && ~isempty(msgGeo))
    if ( ~isempty(msg))
%         subplot(ROS_MACE.altitude);
        plot(ROS_MACE.altitude,time,msg.Altitude,[colors(msg.VehicleID) 'o']);
        hold on;
        if ( time > 30 )
            xlim(ROS_MACE.altitude,[time-30 time])
        end
        drawnow;
        
        % plot position
%         [Easting, Northing,~] = geodetic2enu(msg.Latitude,msg.Longitude,0,ROS_MACE.LatRef,ROS_MACE.LongRef,0,wgs84Ellipsoid,'degrees');
        [xf3, yf3] = ENUtoF3(msg.Easting, msg.Northing);
%         subplot(ROS_MACE.taskAndLocation);
        plot(ROS_MACE.taskAndLocation,xf3,yf3,[colors(msg.VehicleID) 'o']);
        hold on;
        
        drawnow;
        
        % store the new location along with time
        % uncomment the following line for checkout flight (Sheng)
        %     global agentStateHist;
        %     agentStateHist{msg.VehicleID} = [agentStateHist{msg.VehicleID} [time;xf3;yf3;msg.Altitude]];
    end
end
    

