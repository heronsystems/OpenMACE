clear all;
close all;
clc;

ax = 30;
ay = 3;
dx = 5;
xmin = 0;
xmax = 400;
ymin = 0;
ymax = 400;
xcp = xmin+dx/2:dx:xmax-dx/2;
ycp = ymin+dx/2:dx:ymax-dx/2;

inc = 10;
binEdges = 0:inc:180;
theta = diff(binEdges)/2+binEdges(1:end-1);
E = precomputeReducedDistances(xcp, ycp, theta*pi/180, ax, ay);