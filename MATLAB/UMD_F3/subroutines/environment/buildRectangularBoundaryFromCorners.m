function [xpts,ypts] = buildRectangularBoundaryFromCorners(xmin, xmax,ymin, ymax, borderOffset)
% Input: 
%   (nodeX, nodeY) locations
%   borderOffset : value by which to offset the rectangular boundary from
%   min/max node positions
% Output:
%   (xpts,ypts) defining the boundary

% generate rectangular boundary
xpts = [xmin-borderOffset; xmax+borderOffset; xmax+borderOffset; xmin-borderOffset];
ypts = [ymin-borderOffset; ymin-borderOffset; ymax+borderOffset; ymax+borderOffset];
xpts(end+1) = xpts(1);
ypts(end+1) = ypts(1);
end