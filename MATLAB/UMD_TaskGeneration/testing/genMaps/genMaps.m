clear; close all; clc;
rootPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration';
curPath = pwd;
addpath(rootPath);
addpath(curPath);
cd(rootPath)
updatePath;
cd(curPath);

rng(10);


% common to all maps
boxlength = 200;
boxwidth = 200;
dx = 5;
buffer = 0;


% Map 1
% fileName = 'RandallsIsland_Big.osm';
% refX = -300;
% refY = -200;
% removeList = [10,16,29]; %[4,34,35,36,37,61,25];
% angle = 0*pi/180;
% scale = 2;

% %% Map 2
% fileName = 'SilverSpring.osm';
% refX = -750;
% refY = 450;
% removeList = [23,25,16]; %[4,34,35,36,37,61,25];
% scale = 2;
% angle = 0*pi/180;

% %% Map 3
fileName = 'NYC.osm';
refX = 600;
refY = -350;
removeList = []; %[4,34,35,36,37,61,25];
angle = 0*pi/180;
scale = 2;

%% remove some ways
plotOpenStreetMapLabels(fileName, refX, refY, boxlength, boxwidth, angle, dx, scale)
[nodesXY, LatRef, LongRef, G] = loadOpenStreetMapNodesFlex(fileName, refX, refY, boxlength, boxwidth, angle, dx, removeList, scale);

plot(nodesXY(:,1), nodesXY(:,2),'ko','MarkerSize',4)
set(gca,'FontSize',16)
axis equal;
axis tight;
hold on;
xlabel('X (m)')
ylabel('Y (m)')
title('Occupancy Graph')

%save('RandallsIsland.mat');
%save('SilverSpring.mat');
%save('NYC.mat');
