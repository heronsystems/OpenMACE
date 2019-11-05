function [x, y] = generateCircle(xc, yc, R, numPts)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [x, y] = generateCircle(xc, yc, R, numPts)
%   Generates the x,y coordinates of a circle of radius R with a given
%   number of points.
%
% Inputs: xc : x-coordinate of circle center
%         yc : y-coordinate of circle center
%         R : radius of circle
%         numPts : number of points to return
%
% Outputs: x : x-coordinate of circle
%          y : y-coordinate of circle
%
% Artur Wolek                                    Last Modified: 14-Mar-2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
th = linspace(0, 2*pi, numPts);
x = R.*cos(th) + xc;
y = R.*sin(th) + yc;
end
