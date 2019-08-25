function [x,y] = ellipseCoords(ellipseParams, numPts)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [x,y] = ellipseCoords(ellipseParams, numPts)
%   Generate the coordinates of an ellipse.
% 
% Inputs: ellipseParams : a struct with the following fields
%         ellipseParams.xc : x-center of ellipse
%         ellipseParams.yc : y-center of ellipse
%         ellipseParams.semiMinor : semi-minor axis of ellipse
%                                 : half the 
%         ellipseParams.semiMajor : semi-major axis of ellipse
%         ellipseParams.angle : right-handed angle of semi-major axis
%                               measured from x-axis
% 
% Outputs: x : x-coordinates of ellipse
%          y : y-coordinates of ellipse
%
% Artur Wolek                                    Last Modified: 14-Mar-2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The general parametric equation of an ellipse, centered around (xc, yc),
% inclined at angle phi relative to the x-axis
% and with semi major axis a, and semi-minor axis b is:
%
% x(t) = xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi)
% y(t) = yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi)		
%  	
% where: 		0 <= t <= 2*pi

% extract parameters 
xc = ellipseParams.mean(1);
yc = ellipseParams.mean(2);
semiMajor = ellipseParams.semiMajor;
semiMinor = ellipseParams.semiMinor;
angle = ellipseParams.angle;

% generate the curve parametrically
t = linspace(0,2*pi,numPts);
x = xc + semiMajor*cos(t)*cos(angle) - semiMinor*sin(t)*sin(angle);
y = yc + semiMajor*cos(t)*sin(angle) + semiMinor*sin(t)*cos(angle);

end
