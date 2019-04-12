function [ output_args ] = gpsCallback( subscriber, msg )
%LASERSUB Summary of this function goes here
%   Detailed explanation goes here
    global missionStatus;
    i = msg.VehicleID;
    %global missionStatus.armed(i) = 1;
    
%      disp('Message:')
      msg
     
%      subscriber
     
end

