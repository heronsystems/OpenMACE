function [h1, h2] = headingRangeAtCorner(xpoly, ypoly, cornerPt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [h1, h2] = headingRangeAtCorner(xpoly, ypoly, cornerPt)

% Inputs
%   (xpoly, ypoly) : vectors of coordinate defining polygon boundary
%                    xpoly and ypoly are defined CCW and form a loop
%                    i.e., xpoly(1) = xpoly(end) , ypoly(1) = ypoly(end)
%   cornerPt : index of the corner
%
% Output
%   [h1, h2] : two heading angles (in radians) giving the range of angles
%   that point inwards (towards the center of the polygon).
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = length(xpoly);
% define three points (A,B,C) with B being the index of the corner pt, A
% being the preceeding unique corner, and C being the proceeding corner
if (cornerPt == N)
    ptA = N-1;
    ptB = N;
    ptC = 2;
elseif (cornerPt == 1)
    ptA = N-1;
    ptB = 1;
    ptC = 2;
else
    ptA = N-1;
    ptB = N;
    ptC = N+1;
end
% h1 is the heading of the line segment from B to A 
dxBA = xpoly(ptA) - xpoly(ptB);
dyBA = ypoly(ptA) - ypoly(ptB);
h1 = atan2(dxBA, dyBA);
% h2 is the heading of the line segment form B to C
dxBC = xpoly(ptC) - xpoly(ptB);
dyBC = ypoly(ptC) - ypoly(ptB);
h2 = atan2(dxBC, dyBC);
end