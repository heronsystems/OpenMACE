clear; close all; clc;
rootPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration';
curPath = pwd;
addpath(rootPath);
addpath(curPath);
cd(rootPath)
updatePath;
cd(curPath);

%% plot baseline map
%fileName = 'RandallsIsland_Big.osm';
fileName = 'SilverSpring.osm';
%fileName = 'NYC.osm';
refX = -600;
refY = 350;


[ways, LatRef, LongRef] = parseOpenStreetMap(fileName);
figure;
for i = 1:1:length(ways)
    plot(ways{i}(:,1), ways{i}(:,2),'o-')
    hold on;
end
set(gca,'FontSize',16)
axis equal;
axis tight;
hold on;
grid on;

boxlength = 300;
boxwidth = 300;
boxXY = boxCorners(refX,refY,boxlength,boxwidth,0);
plot([boxXY(:,1); boxXY(1,1)],[boxXY(:,2); boxXY(1,2)],'k-','linewidth',2);
xlabel('East')
ylabel('North')
title('Open Street Map')