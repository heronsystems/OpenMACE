function [h1, h2] = headingRangeAlongSeg(prevPt, xpoly, ypoly)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [h1, h2] = headingRangeAlongSeg(prevPt, xpoly, ypoly)

% Inputs
%   (xpoly, ypoly) : vectors of coordinate defining polygon boundary
%                    xpoly and ypoly are defined CCW and form a loop
%                    i.e., xpoly(1) = xpoly(end) , ypoly(1) = ypoly(end)
%   prevPt : the index corresponding to the start of the line segment being
%            queried
%
% Output
%   [h1, h2] : two heading angles (in radians) giving the range of angles
%   that point inwards (towards the center of the polygon). Since
%   this is a line segment the two angles are 180 deg. apart 
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = length(xpoly);
if (prevPt == N)
    nextPt = 2;
elseif (prevPt == 1)
    nextPt = 2;
else
    nextPt = prevPt+1;
end
% angle from nextPt to prevPt
dxNextToPrev = xpoly(prevPt) - xpoly(nextPt);
dyNextToPrev = ypoly(prevPt) - ypoly(nextPt);
h2 = atan2(dyNextToPrev, dxNextToPrev);
% parallel but opposite: from prevPt to nextPt
dxPrevToNext = xpoly(nextPt) - xpoly(prevPt);
dyPrevToNext = ypoly(nextPt) - ypoly(prevPt);
h1 = atan2(dyPrevToNext, dxPrevToNext);

end