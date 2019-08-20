function plotHandles = initLikelihoodWptsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
plotHandles.figh_subplot2 = imagesc(trueWorld.xcp,trueWorld.ycp, log(swarmWorld.samplingPriority) ); %mutualInfoSurface);
set(gca,'YDir','normal')
title('Sampling Priority Surface')
colorbar;
caxis([-10 2])
set(gca,'FontSize',14)
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

% 
for i = 1:1:swarmModel.N
    set(plotHandles.figh_voronoiCenters,'XData',swarmWorld.cellCenterOfMass(:,1), 'YData',swarmWorld.cellCenterOfMass(:,2));
    for j = 1:1:size(swarmState.wptList,1)
        ind = swarmState.wptList(i,j);
        bundleX(j) = swarmWorld.cellCenterOfMass(ind,1);
        bundleY(j) = swarmWorld.cellCenterOfMass(ind,2);        
    end
    plotHandles.figh_bundle(i) = plot(bundleX,bundleY,'mo-','linewidth',2);
end


end