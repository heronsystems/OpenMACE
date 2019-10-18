clear; close all; clc;

figure;
subplot(1,3,1);
load('RandallsIsland.mat');
plot(G,'XData',nodesXY(:,1),'YData',nodesXY(:,2),'MarkerSize',3);
axis equal;
axis square;
xlim([0 boxwidth]);
ylim([0 boxlength]);
title('Randalls Island, NY')

subplot(1,3,2);
load('NYC.mat');
plot(G,'XData',nodesXY(:,1),'YData',nodesXY(:,2),'MarkerSize',3);
axis equal;
axis square;
xlim([0 boxwidth]);
ylim([0 boxlength]);
title('New York City, NY')

subplot(1,3,3);
load('SilverSpring.mat');
plot(G,'XData',nodesXY(:,1),'YData',nodesXY(:,2),'MarkerSize',3);
axis equal;
axis square;
xlim([0 boxwidth]);
ylim([0 boxlength]);
title('Silver Spring, MD')
set(gcf,'Color','w')