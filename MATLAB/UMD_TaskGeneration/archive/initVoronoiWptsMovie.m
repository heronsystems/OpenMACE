function plotHandles = initVoronoiWptsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
plotHandles.figh_subplot2 = imagesc(trueWorld.xcp,trueWorld.ycp,swarmWorld.samplingPriority);
set(gca,'YDir','normal')
title('Sampling Priority')
colorbar;
set(gca,'FontSize',16)
xlabel('X (m)')
ylabel('Y (m)')
hold on;
for i = 1:1:length(swarmWorld.voronoiCells)
    ind = swarmWorld.voronoiCells{i};
    ind = [ind ind(1)];
    plotHandles.figh_voronoiCells(i) = plot(swarmWorld.voronoiVertices(ind,1),swarmWorld.voronoiVertices(ind,2),'r-','linewidth',1);
    hold on;
end
plotHandles.figh_voronoiCenters = plot(swarmWorld.cellCenterOfMass(:,1), swarmWorld.cellCenterOfMass(:,2),'r+','linewidth',1);
axis equal;
if ( runParams.movie.useBackgroundImg )
    xlim([trueWorld.minX-runParams.movie.plotBuffer trueWorld.maxX+runParams.movie.plotBuffer]);
    ylim([trueWorld.minY-runParams.movie.plotBuffer trueWorld.maxY+runParams.movie.plotBuffer]);
else
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
end

end