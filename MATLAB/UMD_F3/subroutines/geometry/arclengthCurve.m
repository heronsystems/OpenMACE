function s = arclengthCurve(x,y,closeCurveFlag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: s = arclengthCurve(x,y,closeCurveFlag)
%
% Inputs
%   (x, y) : two N-sized vectors of coordinates for an arbitrary curve
%   closeCurveFlag : (0 or 1)
%       if 1 the distance from the final point to initial point is added
%       (e.g., if a polygon is specified without repeating points, this
%       gives the closed curve arc-length)
%
% Output
%   s : The total arc-length of the curve computed by summing intermediate
%       straight-line segments.
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = length(x);
dx = diff(x);
dy = diff(y);
ds = (dx.^2+dy.^2).^(1/2);

if ( closeCurveFlag )
    % add the final connection
    dxEnd = x(1) - x(end);
    dyEnd = y(1) - y(end);
    s = sum(ds) + sqrt(dxEnd + dyEnd);
else
    s = sum(ds);
end
end