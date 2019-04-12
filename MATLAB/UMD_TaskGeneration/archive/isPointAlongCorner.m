function status = isPointAlongCorner(x,y,xpoly,ypoly)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: status = isPointAlongCorner(x,y,xpoly,ypoly)
%
% Inputs
%   (x,y) : query point
%   (xpoly,ypoly) : defines polygon boundary
%
% Output
%   status = (1,0) flag indicating if (x,y) is equal to a coordinate in
%   (xpoly,ypoly)
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% find indices where x == xpoly
xEqInd = find(x == xpoly);
if ( isempty(xEqInd) )
    status = 0;
else
    % from this set, find indices where y == ypoly
    if ( any( y == ypoly(xEqInd) ) )
        status = 1;
    else
        status = 0;
    end
end

end