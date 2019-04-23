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
cellStateMat = swarmWorld.cellStateMat;
%figh_subplot2 = surf(trueWorld.xx,trueWorld.yy,cellStateMat,'FaceAlpha',0.5,'EdgeColor','None'); %,abs(2-cellStateMat)./2,'EdgeColor','None');
%view(2)
% plotHandles.figh_coverageMap = imagesc(trueWorld.xcp,trueWorld.ycp,-30*ones(size(cellStateMat)),'AlphaData',abs(cellStateMat)./2);
% set(gca,'YDir','Normal')
% axis tight;

% plot sensing radius
for i = 1:1:swarmModel.N
    xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
    xc = xcnom + xk(1);
    yc = ycnom + xk(2);
    plotHandles.figh_sensingRadius(i) = plot(xc,yc,'m-','LineWidth',2);
end
% plot target locations
for i = 1:1:targetModel.M
    if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
        curNode = targetState.x(4*i-3);
    elseif ( strcmp(targetModel.type, 'constantSpeedRandomWalk') || strcmp(targetModel.type,'constantSpeedRandomWalkGenerative') )
        curNode = targetState.x(2*i-1);
    end
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
    switch swarmModel.mappingSensorType
        case 'perfect'
            xData = trueWorld.G_env.Nodes.x(swarmWorld.exploredGraph.Nodes.trueGraphIndex);
            yData = trueWorld.G_env.Nodes.y(swarmWorld.exploredGraph.Nodes.trueGraphIndex);
        case 'noisy'
            xData = trueWorld.xcp( swarmWorld.exploredGraph.Nodes.bx );
            yData = trueWorld.ycp( swarmWorld.exploredGraph.Nodes.by );
    end
    hold on;
    plot(swarmWorld.exploredGraph,'XData',xData,'YData',yData,'NodeLabel',[]);
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