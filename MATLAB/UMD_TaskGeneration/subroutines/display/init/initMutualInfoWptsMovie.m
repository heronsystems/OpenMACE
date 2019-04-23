function plotHandles = initMutualInfoWptsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
plotHandles.figh_subplot2 = imagesc(trueWorld.xcp,trueWorld.ycp, swarmWorld.samplingPriority );
set(gca,'YDir','normal')
title('Mutual Information Surface')
colorbar;
caxis([0 1])
set(gca,'FontSize',14)
xlabel('X (m)')
ylabel('Y (m)')
hold on;
for i = 1:1:length(swarmWorld.voronoiCells)
    ind = swarmWorld.voronoiCells{i};
    ind = [ind ind(1)];
    plotHandles.figh_voronoiCells(i) = plot(swarmWorld.voronoiVertices(ind,1),swarmWorld.voronoiVertices(ind,2),'-','Color',[1 1 1]*0.9,'linewidth',1);
    hold on;
end
plotHandles.figh_voronoiCenters = plot(swarmWorld.cellCenterOfMass(:,1), swarmWorld.cellCenterOfMass(:,2),'+','Color',[1 1 1]*0.9,'linewidth',1);
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
    for j = 1:1:size(swarmState.wptList,2)%-2
        ind = swarmState.wptList(i,j);
        bundleX(j) = swarmWorld.cellCenterOfMass(ind,1);
        bundleY(j) = swarmWorld.cellCenterOfMass(ind,2);        
    end    
    %xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
    plotHandles.figh_bundle(i) = plot([bundleX],[bundleY],'mo-','linewidth',2,'MarkerFaceColor','w');
end

%
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);

% plot sensing radius
for i = 1:1:swarmModel.N
    switch swarmModel.communicationTopology
        case 'centralized'
            xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
        case 'allToAll'
            xk = [ swarmState{i}.x(1); swarmState{i}.x(2); swarmState{i}.x(3); swarmState{i}.x(4) ];
    end
    xc = xcnom + xk(1);
    yc = ycnom + xk(2);
    plotHandles.figh_sensingRadius(i) = plot(xc,yc,'m-','LineWidth',2);
end

end