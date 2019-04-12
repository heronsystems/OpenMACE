clear; close all; clc;
addpath('../')

% create surface
% dimension of world [x,y]
WORLD = [58 81];
% x and y grids used for generating surface
tt1 = linspace(-WORLD(1)/2,WORLD(1)/2,WORLD(1));
tt2 = linspace(-WORLD(2)/2,WORLD(2)/2,WORLD(2));
[t1,t2] = meshgrid(tt1,tt2);
TP=[];
[T,GX,GY,TP] = tworld(t1,t2,WORLD,TP);
% plot surface
figure(gcf), clf, hold on
%set(gcf,'DoubleBuffer','on')
set(gcf,'WindowButtonDownFcn',@mybuttondownfcn)
set(gca,'Linewidth',2,'Fontsize',18), box on
axis xy, axis([tt1(1) tt1(end) tt2(1) tt2(end)]), axis image
map = flipud(colormap('bone')); colormap(map)
global Thandle, Thandle = imagesc(tt1,tt2,T);


% solve
N = 25;
stepSizeGain = 0.2;
percentTol = 0.05;
maxIters = 100;


[voronoiVertices, voronoiCells, cellMass, cellCenterOfMass] = approxEqualMassVornoiPartition(t1,t2,T,N,stepSizeGain,percentTol,maxIters);

% plot results
for i = 1:1:length(voronoiCells)
    ind = voronoiCells{i};
    ind = [ind ind(1)];
    plot(voronoiVertices(ind,1),voronoiVertices(ind,2),'go-','linewidth',2);
    hold on;
end
plot(cellCenterOfMass(:,1), cellCenterOfMass(:,2),'r+','linewidth',2)
axis([tt1(1) tt1(end) tt2(1) tt2(end)])