function plotHandles  = initLikelihoodOccupancyGraphExploredMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
% movie
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);
% plot background
if ( runParams.movie.useBackgroundImg )
    img = imread(runParams.movie.backgroundImgFile);
    xdataVec = [runParams.movie.backgroundImgBottomLeftCornerX runParams.movie.backgroundImgBottomLeftCornerX+runParams.movie.backgroundImgWidth];
    ydataVec = [runParams.movie.backgroundImgBottomLeftCornerY runParams.movie.backgroundImgBottomLeftCornerY+runParams.movie.backgroundImgHeight];
    image(flipud(img), 'XData', xdataVec, 'YData', ydataVec );
    set(gca,'YDir','Normal')
    axis equal;
    xlim([trueWorld.minX-runParams.movie.plotBuffer trueWorld.maxX+runParams.movie.plotBuffer]);
    ylim([trueWorld.minY-runParams.movie.plotBuffer trueWorld.maxY+runParams.movie.plotBuffer]);
else
    set(gcf,'color','white')
    set(gca, 'color', [0.8 0.8 0.8])
    axis equal;
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
end
%
hold on;

% plot sensing radius
for i = 1:1:swarmModel.N
    xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
    xc = xcnom + xk(1);
    yc = ycnom + xk(2);
    plotHandles.figh_sensingRadius(i) = plot(xc,yc,'m-','LineWidth',2);
end
% plot target locations
for i = 1:1:targetModel.M
curNode = targetState.x(2*i-1);
    targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
    plotHandles.figh_targetLoc(i) = plot(targXY(1), targXY(2), 'r+','linewidth',2);
end
% legends, axes
set(gca,'FontSize',14)
if ( ~strcmp(trueWorld.type,'osmAtF3') )
    xlabel('X(m)')
end
ylabel('Y(m)')
title('Occupancy Graph')

if ( ~isempty(swarmWorld.exploredGraph.Nodes) )
    %xData = trueWorld.xcp( swarmWorld.exploredGraph.Nodes.bx );
    %yData = trueWorld.ycp( swarmWorld.exploredGraph.Nodes.by );
    hold on;
    plot(swarmWorld.exploredGraph,'XData',swarmWorld.exploredGraph.Nodes.nodeX,'YData',swarmWorld.exploredGraph.Nodes.nodeY,'NodeLabel',[]);
end
colormap(gca,'parula')
colorbar;
caxis([-10 1]);
%caxis([0 1]);
hold off;

% plot background
if ( runParams.movie.useBackgroundImg )
    xlim([trueWorld.minX-runParams.movie.plotBuffer trueWorld.maxX+runParams.movie.plotBuffer]);
    ylim([trueWorld.minY-runParams.movie.plotBuffer trueWorld.maxY+runParams.movie.plotBuffer]);
end

end