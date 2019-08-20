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
boxlength = 300;
boxwidth = 300;
dx = 5;
buffer = 0;


%% Map 1
% fileName = 'RandallsIsland_Big.osm';
% refX = -300;
% refY = -200;
% removeList = [10,16,29]; %[4,34,35,36,37,61,25];
%angle = 0*pi/180;

% %% Map 2
% fileName = 'SilverSpring.osm';
% refX = -620;
% refY = 450;
% removeList = []; %[4,34,35,36,37,61,25];
%angle = 0*pi/180;

% %% Map 3
fileName = 'NYC.osm';
refX = 600;
refY = -350;
removeList = []; %[4,34,35,36,37,61,25];
angle = 60*pi/180;

%% remove some ways
plotOpenStreetMapLabels(fileName, refX, refY, boxlength, boxwidth, angle, dx)
[nodesXY, LatRef, LongRef, G] = loadOpenStreetMapNodesFlex(fileName, refX, refY, boxlength, boxwidth, angle, dx, removeList);

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
