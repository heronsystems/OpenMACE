function plotHandles  = initGroundTruthMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
% movie
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);
% create pixel matrix
params.minXpixel = floor(trueWorld.minX);
params.minYpixel = floor(trueWorld.minY);
params.maxXpixel = ceil(trueWorld.maxX);
params.maxYpixel = ceil(trueWorld.maxY);
params.width = params.maxXpixel - params.minXpixel;
params.height = params.maxYpixel - params.minYpixel;

L = swarmModel.Rsense;


if ( runParams.movie.useBackgroundImg )
    img = imread(runParams.movie.backgroundImgFile);
    xdataVec = [runParams.movie.backgroundImgBottomLeftCornerX runParams.movie.backgroundImgBottomLeftCornerX+runParams.movie.backgroundImgWidth];
    ydataVec = [runParams.movie.backgroundImgBottomLeftCornerY runParams.movie.backgroundImgBottomLeftCornerY+runParams.movie.backgroundImgHeight];
    image(flipud(img), 'XData', xdataVec, 'YData', ydataVec );
    set(gca,'YDir','Normal')
end

hold on;
% plot occupancy graph
plotHandles.p1 = plot(trueWorld.G_env,'XData',trueWorld.G_env.Nodes.x,'YData',trueWorld.G_env.Nodes.y);
plotHandles.p1.MarkerSize = 4;

% plot sensing radius
for i = 1:1:swarmModel.N
    xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
    xc = xcnom + xk(1);
    yc = ycnom + xk(2);
    plotHandles.figh_sensingRadius(i) = plot(xc,yc,'k-');
end
% % plot target locations
% for i = 1:1:targetModel.M
% curNode = targetState.x(2*i-1);
%     targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
%     plotHandles.figh_targetLoc(i) = plot(targXY(1), targXY(2), 'r+','linewidth',2);
% end
% plot F3 boundary
if ( runParams.movie.plotF3Obstacles )
    plot(runParams.movie.perimX, runParams.movie.perimY, 'k-','Linewidth',2);
    plot(runParams.movie.pole1x, runParams.movie.pole1y, 'k-','Linewidth',2);
    plot(runParams.movie.pole2x, runParams.movie.pole2y, 'k-','Linewidth',2);
end

% plot background
if ( runParams.movie.useBackgroundImg )
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
% legends, axes
set(gca,'FontSize',16)
if ( ~strcmp(trueWorld.type,'osmAtF3') )
    xlabel('X(m)')
end
ylabel('Y(m)')
%title(['Ground Truth']);
% colorbar;
% caxis([-30 30]);
hold off;

end