%
clear; close all; clc;
run('../updatePath.m')

% % right plot : manipulated map with edges
fileName = 'RandallsIsland_Big.osm';
refX = -300;
refY = -200;
angle = 0*pi/180;
boxlength = 400;
boxwidth = 400;
dx = 5;
buffer = 1;

%% remove some ways
plotOpenStreetMapLabels(fileName, refX, refY, boxlength, boxwidth, angle, dx)
removeList = [4,3,34,35,36,37,7,61,25];
[nodesXY, LatRef, LongRef, G] = loadOpenStreetMapNodesFlex(fileName, refX, refY, boxlength, boxwidth, angle, dx, removeList);
