clear; close all; clc;
%load('data/RandallsIsland_Big.osm_openStreetMap_full.mat')
%load('data/cityblocksAtF3_cityblocksAtF3_full.mat');
load('data/randomRoadsAtF3_randomRoadsAtF3_full.mat');

addpath('/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration');
addpath('/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration/mex');
%updatePath;

figure(1);
plot(trueWorld.nodeX, trueWorld.nodeY,'ko','MarkerFaceColor','k'); hold on;
axis equal;
xlim([trueWorld.minX trueWorld.maxX]);
ylim([trueWorld.minY trueWorld.maxY]);

numSamples = 30;
R = 1.25;
%R = 30;
ax = 1.5;
ay =0.1;

% plot footprints
th = linspace(0,2*pi);
xc = R*cos(th);
yc = R*sin(th);
dx = trueWorld.xcp(2) - trueWorld.xcp(1);
dy = trueWorld.ycp(2) - trueWorld.ycp(1);
windowWidth = 3*ceil(R/dx)+1;
halfWidth = floor((windowWidth-1)/2);

cellsInView = [];
for i = 1:1:numSamples
    xr(i) = rand()*(trueWorld.maxX - trueWorld.minX) + trueWorld.minX;
    yr(i) = rand()*(trueWorld.maxY - trueWorld.minY) + trueWorld.minY;
    plot(xr(i) + xc, yr(i) + yc, 'r-');
    agent = [xr(i) yr(i)];
    [cellsInView] = [cellsInView; findCellsInView(agent,trueWorld.xcp,trueWorld.ycp,dx,dy,trueWorld.numBinsX,trueWorld.numBinsY,halfWidth,R)];
    for i = 1:1:size(cellsInView,1)
        bx = cellsInView(i,1);
        by = cellsInView(i,2);
        plot(trueWorld.xcp(bx), trueWorld.ycp(by), 'b.');
    end
end
cellsInView = unique(cellsInView,'rows');

% plot sensed
figure(2);
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
    axis equal;
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
end
for i = 1:1:size(cellsInView,1)
    bx = cellsInView(i,1);
    by = cellsInView(i,2);
    measurements(i,1) = trueWorld.xcp(by);
    measurements(i,2) = trueWorld.ycp(bx);
    
    if (trueWorld.bin2NodeID(bx,by)==0)
        plot(trueWorld.xcp(bx), trueWorld.ycp(by), 'b.');
        measurements(i,3) = 0;
    else
        plot(trueWorld.xcp(bx), trueWorld.ycp(by), 'ko','MarkerFaceColor','k');
        measurements(i,3) = 1;
    end
end
krigingSigma = 1;

forecast = ordinaryKrig(trueWorld.xx,trueWorld.yy,measurements,krigingSigma);


figure;
imagesc(forecast','XData', trueWorld.xcp, 'YData', trueWorld.ycp); hold on;
set(gca,'YDir','Normal');
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
end
axis equal;
xlim([trueWorld.minX trueWorld.maxX]);
ylim([trueWorld.minY trueWorld.maxY]);




forecast = anisotropicKrig(trueWorld.xx,trueWorld.yy,measurements,ax,ay);


figure;
imagesc(forecast','XData', trueWorld.xcp, 'YData', trueWorld.ycp); hold on;
set(gca,'YDir','Normal');
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
end
axis equal;
xlim([trueWorld.minX trueWorld.maxX]);
ylim([trueWorld.minY trueWorld.maxY]);