function [xpoly,ypoly] = buildConvexBoundary(nodeX, nodeY, borderOffset, borderScale)
Npts = length(nodeX);
% define maximal boundaries of survey area
xmin = min(nodeX);
xmax = max(nodeX);
ymin = min(nodeY);
ymax = max(nodeY);
% generate an arbitrary convex polygon
xpts = [xmin-borderOffset + (rand(Npts,1))*(2*borderOffset+xmax-xmin); xmin*borderScale; xmax*borderScale; xmin*borderScale; xmax*borderScale];
ypts = [ymin-borderOffset + (rand(Npts,1))*(2*borderOffset+ymax-ymin); ymin*borderScale; ymax*borderScale; ymax*borderScale; ymin*borderScale];
[chInd] = convhull( xpts, ypts ); % returns a CCW cycle
xpoly = xpts(chInd);
ypoly = ypts(chInd);
end
