clear all; close all; clc;

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


numCells = 4;
%[voronoiVertices, voronoiCells, cellMass, cellCenterOfMass] = equalAreaVoronoi3(xx, yy, numCells)

[xCM, yCM, cellMass, Ac] = equalAreaVoronoiMod(xx, yy, numCells)
% 
% for i = 1:1:length(voronoiCells)
%     ind = voronoiCells{i};
%     ind = [ind ind(1)];
%     plot(voronoiVertices(ind,1),voronoiVertices(ind,2),'mo-','linewidth',2);
%     hold on;
% end
% plot(cellCenterOfMass(:,1), cellCenterOfMass(:,2),'m+','linewidth',2);
% axis equal;
% xlim([xmin xmax])
% ylim([ymin ymax])
% xlabel('X (m)')
% ylabel('Y (m)')
% set(gca,'FontSize',16)