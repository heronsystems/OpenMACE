% test_lawnMower
clear; close all; clc;
format compact;

% Algorithm outline:
% 1. Compute equal area voronoi partition
% 2. Determine direction of each edge where agent is located
%   (ignore corner case)
% 3. Direction perpendicular to edge defines survey direction
% 4. Determine if closest point on survey cell is left/right
% 5. Determine direction of survey progression
% 6. Discretize length of survey into number of lines determined by
%    sensing radius
% 7. compute ymin and ymax at each discretized point, use to define
%    a waypoint sequence.


% boundaries of area
xmin = 0;
xmax = 100;
ymin = 0;
ymax = 50;

% define surface for voronoi partition
numPts = 100;
xrange = linspace(xmin, xmax, numPts);
yrange = linspace(ymin, ymax, numPts);
[xx,yy] = meshgrid(xrange,yrange);
zz = xx;
imagesc(zz)



%
numCells = 5;
% sites
xs = xmin + rand(numCells,1)*(xmax-xmin);
ys = ymin + rand(numCells,1)*(ymax-ymin);

plotFlag = 1;
maxIters = 10;
percentTol = 0.001;
[xCM, yCM, cellMass, Ac] = equalAreaVoronoi(xx, yy, zz, numCells, xs, ys, maxIters, percentTol, plotFlag);


figure;
imagesc('XData',[xmin xmax],'YData',[ymin ymax],'CData',Ac)
set(gca,'YDir','Normal')
hold on;
axis equal;
plot(xCM,yCM,'c+','linewidth',2)


% % voronoi algorithm
% numCells = 4;
% stepSizeGain = 0.2;
% percentTol = 0.005;
% maxIters = 500;
% startPts(1:numCells,1) = xmin + rand(numCells,1)*(xmax-xmin);
% startPts(1:numCells,2) = ymin + rand(numCells,1)*(ymin-ymax);
% [voronoiVertices, voronoiCells, cellMass, cellCenterOfMass] = approxEqualMassVornoiPartition(xx,yy,zz,numCells,stepSizeGain,percentTol,maxIters, startPts)
%
% % plot
% figure;
% for i = 1:1:numCells
%     ind = voronoiCells{i};
%     ind = [ind ind(1)];
%     plot(voronoiVertices(ind,1),voronoiVertices(ind,2),'r-','linewidth',1);
%     hold on;
% end
% axis equal;
% xlim([xmin xmax])
% ylim([ymin ymax])

