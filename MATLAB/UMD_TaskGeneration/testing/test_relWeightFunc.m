% test_relWeightFunc.m
% 29-Sep-2018, A. Wolek
% Script to plot relative weight function used to compute state transition
% matrix probability based on difference in heading angle between two
% connected states.

clear;
close all;
clc;

% assume this .m file is launching from ./testing folder
addpath('./')
cd ..
updatePath;

% weight function params
d1 = 0.1;
d2 = 1;
d3 = 25;
m = 1;

% domain
x = linspace(0,pi);

% parametrized curves
y1 = relWeightFun(x,m,d1);
y2 = relWeightFun(x,m,d2); %y(b2,x);
y3 = relWeightFun(x,m,d3);

% plot
figure;
p1 = plot(x*180/pi,y1,'linewidth',2);
hold on;
p2 = plot(x*180/pi,y2,'linewidth',2);
p3 = plot(x*180/pi,y3,'linewidth',2);
plot(0,1+m,'ko','linewidth',2)
hold on;
plot(pi*180/pi,1,'ko','linewidth',2)
plot([0 180],[1 1],'k--')
xlim([0 pi]*180/pi)
ylim([0 1+m])
set(gca,'FontSize',16)
ylabel('Relative Weight')
xlabel('\Delta \theta (deg)')
title(['m = ' num2str(m)])
legend([p1 p2 p3],{['d = ' num2str(d1)],['d = ' num2str(d2)],['d = ' num2str(d3)]} );
grid on;

