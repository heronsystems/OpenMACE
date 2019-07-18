clear; close all; clc;
%load('data/RandallsIsland_Big.osm_openStreetMap_full.mat')
%load('data/cityblocksAtF3_cityblocksAtF3_full.mat');
%load('data/randomRoadsAtF3_randomRoadsAtF3_full.mat');
%load('data/RandallsIsland_Big.osm_osmAtF3_full.mat')

addpath('/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration');
addpath('/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration/mex');
updatePath;
fileName = './data/RandallsIsland_Big.osm';


%% load data
nodeFactor = 1;
dim = 1;
refX = -300;
refY = -200;
angle = 0*pi/180;
boxlength = 400;
boxwidth = 400;
dx = 5;
buffer = 1;
[nodesXY] = loadOpenStreetMapFlexNodes(fileName, refX, refY, boxlength, boxwidth, angle, dx, buffer);

trueWorld.minX = 0;
trueWorld.maxX = 400;
trueWorld.minY = 0;
trueWorld.maxY = 400;


%% plot nodes/edges
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

%% sample data
numSamples = 20;
R = 30;
ax = R;
ay =0.1*R;

% plot footprints
th = linspace(0,2*pi);
xc = R*cos(th);
yc = R*sin(th);


nodeX = nodesXY(:,1);
nodeY = nodesXY(:,2);

xmin = 0;
xmax = 400;
ymin = 0;
ymax = 400;

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
figure;
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
    axis equal;
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
end
for i = 1:1:size(cellsInView,1)
    bx = cellsInView(i,1);
    by = cellsInView(i,2);
    measurements(i,1) = xcp(bx);
    measurements(i,2) = ycp(by);
    
    if (grid(by,bx)==0)
        plot(xcp(bx), ycp(by), 'b.');
        measurements(i,3) = 0;
    else
        plot(xcp(bx), ycp(by), 'ko','MarkerFaceColor','k');
        measurements(i,3) = 1;
    end
end


%% analyze edges histogram
figure;
for i = 1:1:length(edgeDir)
   if (edgeDir(i) < 0)
      edgeDir(i) = pi+edgeDir(i); 
   end
end
inc = 3;
nbins = floor(180/inc);
histogram(edgeDir*180/pi,nbins)
[N,bins] = histcounts(edgeDir*180/pi,nbins)
theta = diff(bins)/2+bins(1:end-1);
sep = inc;
npeaks = 4;


dirMag(1,:) = N;
for i = 1:1:npeaks
    [maxVal, maxInd] = max(dirMag(i,:));
    peaks(i) = theta(maxInd);
    weight(i) = 0;
    dirMag(i+1,:) = dirMag(i,:);
    % modify dirMag
    if (maxInd >= sep ) 
        weight(i) = weight(i) + sum(dirMag(i+1,maxInd-sep:maxInd));
        dirMag(i+1,maxInd-sep:maxInd) = 0;        
    else
        weight(i) = weight(i) + sum(dirMag(i+1,1:sep));
        dirMag(i+1,1:sep) = 0;
        weight(i) = weight(i) + sum(dirMag(i+1,end-sep:end));
        dirMag(i+1,end-sep:end) = 0;
    end    
    if (maxInd+sep >= length(dirMag) )
        weight(i) = weight(i) + sum(dirMag(i+1,maxInd-sep:end));
        dirMag(i+1,end-sep:end) = 0;
        weight(i) = weight(i) + sum(dirMag(i+1,1:maxInd-sep));
        dirMag(i+1,1:maxInd-sep) = 0;        
    else
        weight(i) = weight(i) + sum(dirMag(i+1,maxInd:maxInd+sep));
        dirMag(i+1,maxInd:maxInd+sep) = 0;
    end
    %plot(theta,dirMag(i+1,:));
    hold on;
    peakVal(i) = maxVal;
    plot(peaks(i),peakVal(i),'k*','linewidth',2,'MarkerSize',5);  
    xlim([0 180]);
end
set(gca,'FontSize',16)
xlabel('Angle of Edges (deg.)')
ylabel('Number of Edges')
weightnorm = weight/sum(weight);
for i = 1:1:npeaks
    text(peaks(i),peakVal(i)+max(weight)*0.01,num2str(weightnorm(i)));  
end


[xx,yy] = meshgrid(xcp,ycp);

%% ordinary krig

krigingSigma = R;
forecast = ordinaryKrig(xx,yy,measurements,krigingSigma);
    forecast( forecast > 1 ) = 1;
    forecast( forecast < 0 ) = 0;
figure;
imagesc(forecast,'XData', xcp, 'YData', ycp); hold on;
set(gca,'YDir','Normal');
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
end
title('Ordinary Kriging')
set(gca,'FontSize',16)
xlabel('X')
ylabel('Y')
axis equal;
xlim([trueWorld.minX trueWorld.maxX]);
ylim([trueWorld.minY trueWorld.maxY]);
colorbar;

%% anisotropic krig
forecast = anisotropicKrig(xx,yy,measurements,ax,ay);
figure;
imagesc(forecast,'XData', xcp, 'YData', ycp); hold on;
set(gca,'YDir','Normal');
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
end
axis equal;
title('0/90 Anisotropic Kriging')
set(gca,'FontSize',16)
xlabel('X')
ylabel('Y')
xlim([trueWorld.minX trueWorld.maxX]);
ylim([trueWorld.minY trueWorld.maxY]);

%% adaptive anisotropic krig
forecast = anisotropicAdaptiveKrig(xx,yy,measurements,ax,ay, peaks, weight);
figure;
imagesc(forecast,'XData', xcp, 'YData', ycp); hold on;
set(gca,'YDir','Normal');
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
end
axis equal;
title('Adaptive (4-Peak) Anisotropic Kriging')
set(gca,'FontSize',16)
xlabel('X')
ylabel('Y')
xlim([trueWorld.minX trueWorld.maxX]);
ylim([trueWorld.minY trueWorld.maxY]);
colorbar;
