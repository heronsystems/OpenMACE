function [minDist, segPtA, segPtB ] = distToPolygon(x, y, xpoly, ypoly)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [minDist, segPtA, segPtB ] = distToPolygon(x, y, xpoly, ypoly)
%
% Inputs
%   (x,y) : the query point
%   (xpoly,ypoly) : vectors defining the polygon boundary. It is assumed
%   that the first and last point are repeated.
%
% Output
%   minDist : minimum (signed) distanced to the polygon. Points outside the
%             polygon are negative.
%   (segPtA, segPtB) : the minimum distance occurs between these two
%   indices
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialize dist to line
N = length(xpoly);
distToLine = zeros(N-1,1);
% compute distance to each segment of polygon
for i = 1:1:N-1
    [a,b,c] = lineCoeffs( xpoly(i) , ypoly(i) , xpoly(i+1) , ypoly(i+1) );
    distToLine(i) = distToLine2D(a,b,c, x, y );
end
% find the minimum
[minDist, segPtA] = min(distToLine);
% determine corresponding indices
if ( segPtA == N )
    segPtB = 2;
else
    segPtB = segPtA+1;
end
% assign sign
if ( ~inpolygon(x,y,xpoly,ypoly) )
    minDist = -minDist;
end
end