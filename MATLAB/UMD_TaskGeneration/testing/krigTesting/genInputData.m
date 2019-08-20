clear; close all; clc;
rootPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration';
curPath = pwd;
addpath(rootPath);
addpath(curPath);
cd(rootPath)
updatePath;
cd(curPath);

rng(10);

%% load data
fileName = './../data/RandallsIsland_Big.osm';
nodeFactor = 1;
dim = 1;
refX = -300;
refY = -200;
angle = 0*pi/180;
boxlength = 200;
boxwidth = 200;
dx = 5;
buffer = 0;
scale = 1;
numSamples = 10;
R = 3*dx;
ax = R;
ay =0.1*R;


%%
[nodesXY] = loadOpenStreetMapFlexNodes(fileName, refX, refY, boxlength, boxwidth, angle, dx, buffer,scale);
trueWorld.minX = 0;
trueWorld.maxX = boxlength;
trueWorld.minY = 0;
trueWorld.maxY = boxwidth;

%% plot nodes/edges
figure;
edgeFactor = sqrt(2)*1.5;
k = 1;
for i = 1:1:length(nodesXY)
    % calculate distance to all other nodes
    dvec = (nodesXY(i,:) - nodesXY);
    d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
    ind = find(d <= edgeFactor*dx);
    if ( length(ind) >= 2 )
        for j = 1:1:length(ind)
            if ind(j) ~= i
                % draw edge
                plot( [nodesXY(i,1) nodesXY(ind(j),1)],[nodesXY(i,2) nodesXY(ind(j),2)],'k-')
                hold on;
                k = k + 1;
            end
        end
    end
end
plot(nodesXY(:,1), nodesXY(:,2),'ko','MarkerSize',4)
set(gca,'FontSize',16)
axis equal;
axis tight;
hold on;
xlabel('X (m)')
ylabel('Y (m)')
title('Occupancy Graph')
% 


% plot footprints
th = linspace(0,2*pi);
xc = R*cos(th);
yc = R*sin(th);


nodeX = nodesXY(:,1);
nodeY = nodesXY(:,2);

xmin = 0;
xmax = boxwidth;
ymin = 0;
ymax = boxlength;

[xcp, ycp, grid, numBinsX, numBinsY] = imposeGridOnNodes(nodeX, nodeY, xmin, xmax, ymin, ymax, dx)
dy = dx;
windowWidth = 3*ceil(R/dx)+1;
halfWidth = floor((windowWidth-1)/2);
nodesXYinView = [];
cellsInView = [];
for i = 1:1:numSamples
    xr(i) = rand()*(trueWorld.maxX - trueWorld.minX) + trueWorld.minX;
    yr(i) = rand()*(trueWorld.maxY - trueWorld.minY) + trueWorld.minY;
    plot(xr(i) + xc, yr(i) + yc, 'r-');
    agent = [xr(i) yr(i)];
    
    [cellsInView] = [cellsInView; findCellsInView(agent,xcp,ycp,dx,dy,numBinsX,numBinsY,halfWidth,R)];
    for i = 1:1:size(cellsInView,1)
        bx = cellsInView(i,1);
        by = cellsInView(i,2);
        plot(xcp(bx), ycp(by), 'b.');                   
    end
    for i = 1:1:length(nodesXY)
        delX = nodesXY(i,1) - agent(1);
        delY = nodesXY(i,2) - agent(2);
        d = sqrt(delX.^2+delY.^2);
        if ( d <= R)
            nodesXYinView = [nodesXYinView; nodesXY(i,:)];
        end
    end
end
cellsInView = unique(cellsInView,'rows');
nodesXYinView = unique(nodesXYinView,'rows');

%% determine edges in veiw
figure;
k = 1;
for i = 1:1:length(nodesXYinView)
    % calculate distance to all other nodes
    dvec = (nodesXYinView(i,:) - nodesXYinView);
    d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
    ind = find(d <= edgeFactor*dx);
    if ( length(ind) >= 2 )
        for j = 1:1:length(ind)
            if ind(j) ~= i
                % draw edge
                plot( nodesXYinView(i,1), nodesXYinView(i,2),'ko');
                plot( [nodesXYinView(i,1) nodesXYinView(ind(j),1)],[nodesXYinView(i,2) nodesXYinView(ind(j),2)],'k-')
                hold on;j
                plot( nodesXYinView(j,1), nodesXYinView(j,2),'ko');
                delx = nodesXYinView(ind(j),1) - nodesXYinView(i,1);
                dely = nodesXYinView(ind(j),2) - nodesXYinView(i,2);
                edgeDir(k) = atan2(dely,delx);
                k = k + 1;
            end
        end
    end
end
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-');
end


%% package and plot kriging input data
%figure;
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
    axis equal;
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
end
for i = 1:1:size(cellsInView,1)
    bx = cellsInView(i,1);
    by = cellsInView(i,2);
    measurements(i,1) = bx;
    measurements(i,2) = by;
    
    if (grid(by,bx)==0)
        plot(xcp(bx), ycp(by), 'b.');
        measurements(i,3) = 0;
    else
        plot(xcp(bx), ycp(by), 'ko','MarkerFaceColor','k');
        measurements(i,3) = 1;
    end
end
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
set(gcf,'Color','w')
tic 

save krigData;
