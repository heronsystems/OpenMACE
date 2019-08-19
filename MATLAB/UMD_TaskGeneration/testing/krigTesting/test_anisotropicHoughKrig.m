clear; close all; clc;
%load('data/RandallsIsland_Big.osm_openStreetMap_full.mat')
%load('data/cityblocksAtF3_cityblocksAtF3_full.mat');
%load('data/randomRoadsAtF3_randomRoadsAtF3_full.mat');
%load('data/RandallsIsland_Big.osm_osmAtF3_full.mat')

function [nodesXY, LatRef, LongRef] = loadOpenStreetMap(fileName, dx, nodeFactor, edgeFactor, dim )

addpath('/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration');
addpath('/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration/mex');
%updatePath;

figure(1);
plot(trueWorld.nodeX, trueWorld.nodeY,'ko','MarkerFaceColor','k'); hold on;
axis equal;
xlim([trueWorld.minX trueWorld.maxX]);
ylim([trueWorld.minY trueWorld.maxY]);

numSamples = 50;
R = 3;
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
BW = zeros(trueWorld.numBinsX, trueWorld.numBinsY);

for i = 1:1:numSamples
    xr(i) = rand()*(trueWorld.maxX - trueWorld.minX) + trueWorld.minX;
    yr(i) = rand()*(trueWorld.maxY - trueWorld.minY) + trueWorld.minY;
    plot(xr(i) + xc, yr(i) + yc, 'r-');
    agent = [xr(i) yr(i)];
    [cellsInView] = [cellsInView; findCellsInView(agent,trueWorld.xcp,trueWorld.ycp,dx,dy,trueWorld.numBinsX,trueWorld.numBinsY,halfWidth,R)];
    for j = 1:1:size(cellsInView,1)
        bx = cellsInView(j,1);
        by = cellsInView(j,2);
        
        plot(trueWorld.xcp(bx), trueWorld.ycp(by), 'b.');
        if (trueWorld.bin2NodeID(by,bx)~=0)
            BW(bx,by) = 1;
        end
    end
    
end

cellsInView = unique(cellsInView,'rows');

% figure;
% imagesc(BW);
% colorbar;
% 
figure;
dth = 0.1;
[H,theta,rho] = hough(BW,'Theta',linspace(-90,89,floor(189/dth)));
imagesc(H,'XData',theta,'YData',rho);
xlabel('\theta')
ylabel('\rho');
axis on, axis normal;
%colormap(gca,hot)

dirMag = max(H,[],1);
npeaks = 5;
sep = floor(30/dth);
figure;
plot(theta,dirMag); hold on;
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
    plot(theta,dirMag(i+1,:));
    hold on;
    plot(peaks(i),maxVal,'b*','linewidth',2);  
    xlim([-90 90]);
end

peaks
weight

% 
% [featureVector,hogVisualization] = extractHOGFeatures(BW);
% figure;
% imagesc(BW);
% hold on;
% plot(hogVisualization);


% %
% [Gmag,Gdir] = imgradient(BW);
% gdirvec = reshape(Gdir,[size(Gdir,1)*size(Gdir,2),1]);
% plot(gdirvec);

% % plot sensed
% figure(2);
% for i = 1:1:numSamples
%     plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
%     axis equal;
%     xlim([trueWorld.minX trueWorld.maxX]);
%     ylim([trueWorld.minY trueWorld.maxY]);
% end
% for i = 1:1:size(cellsInView,1)
%     bx = cellsInView(i,1);
%     by = cellsInView(i,2);
%     measurements(i,1) = trueWorld.xcp(bx);
%     measurements(i,2) = trueWorld.ycp(by);
%
%     if (trueWorld.bin2NodeID(by,bx)==0)
%         plot(trueWorld.xcp(bx), trueWorld.ycp(by), 'b.');
%         measurements(i,3) = 0;
%     else
%         plot(trueWorld.xcp(bx), trueWorld.ycp(by), 'ko','MarkerFaceColor','k');
%         measurements(i,3) = 1;
%     end
% end
% krigingSigma = 1;
%
% forecast = ordinaryKrig(trueWorld.xx,trueWorld.yy,measurements,krigingSigma);
%
%
% figure;
% imagesc(forecast,'XData', trueWorld.xcp, 'YData', trueWorld.ycp); hold on;
% set(gca,'YDir','Normal');
% for i = 1:1:numSamples
%     plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
% end
% axis equal;
% xlim([trueWorld.minX trueWorld.maxX]);
% ylim([trueWorld.minY trueWorld.maxY]);
%
%
%
%
% forecast = anisotropicKrig(trueWorld.xx,trueWorld.yy,measurements,ax,ay);
%
%
% figure;
% imagesc(forecast,'XData', trueWorld.xcp, 'YData', trueWorld.ycp); hold on;
% set(gca,'YDir','Normal');
% for i = 1:1:numSamples
%     plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
% end
% axis equal;
% xlim([trueWorld.minX trueWorld.maxX]);
% ylim([trueWorld.minY trueWorld.maxY]);