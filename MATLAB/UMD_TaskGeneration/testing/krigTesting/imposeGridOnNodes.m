function [xcp, ycp, grid, numBinsX, numBinsY] = imposeGridOnNodes(nodesX, nodesY, xmin, xmax, ymin, ymax, dx);

nodesX = round(nodesX./dx)*dx + dx/2;
nodesY = round(nodesY./dx)*dx + dx/2;

% center points of grid cells
xcp = xmin+dx/2:dx:xmax-dx/2;
ycp = ymin+dx/2:dx:ymax-dx/2;

%
numBinsX = floor((xmax - xmin) / dx);
numBinsY = floor((ymax - ymin) / dx);

% figure;
% drawgrid(xmin,numBinsX,ymin,numBinsY,dx)
% plot(nodesX, nodesY,'ko');

grid = zeros(numBinsX,numBinsY);
for i = 1:1:length(nodesX)
    bx = (nodesX(i)-dx/2)/dx+1;
    by = (nodesY(i)-dx/2)/dx+1;
    grid(by,bx) = 1;   
end

% figure;
% imagesc(grid);

end