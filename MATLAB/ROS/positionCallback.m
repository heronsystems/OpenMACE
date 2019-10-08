function [ output_args ] = positionCallback( subscriber, msg )
%LASERSUB Summary of this function goes here
%   Detailed explanation goes here

     X = sprintf('UPDATE_POSITION: %f %f %f',msg.Northing,msg.Easting,msg.Altitude);
     disp(X);

%      plot(msg.Northing, msg.Easting,'r*')
%      hold on
end

