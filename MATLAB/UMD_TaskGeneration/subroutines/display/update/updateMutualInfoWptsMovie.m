function plotHandles  = updateMutualInfoWptsMovie( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)

subplot(plotHandles.subplotHandle)
% update sampling priority
set(plotHandles.figh_subplot2,'CData', swarmWorld.samplingPriority );
for i = 1:1:length(swarmWorld.voronoiCells)
    ind = swarmWorld.voronoiCells{i};
    ind = [ind ind(1)];
    set(plotHandles.figh_voronoiCells(i),'XData',swarmWorld.voronoiVertices(ind,1),'YData',swarmWorld.voronoiVertices(ind,2));
end
set(plotHandles.figh_voronoiCenters,'XData',swarmWorld.cellCenterOfMass(:,1), 'YData',swarmWorld.cellCenterOfMass(:,2));

%
for i = 1:1:swarmModel.N
    set(plotHandles.figh_voronoiCenters,'XData',swarmWorld.cellCenterOfMass(:,1), 'YData',swarmWorld.cellCenterOfMass(:,2));
    for j = 1:1:size(swarmState.wptList,2)%-2
        ind = swarmState.wptList(i,j);
        bundleX(j) = swarmWorld.cellCenterOfMass(ind,1);
        bundleY(j) = swarmWorld.cellCenterOfMass(ind,2);
    end
    % add agent current poosition
    xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
    %plot([xk(1) bundleX],[xk(2) bundleY],'mo-','linewidth',2,'MarkerFaceColor','w');
    set(plotHandles.figh_bundle(i),'XData',[bundleX],'YData',[bundleY]);
end

%
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);

% plot sensing radius
% loop through updates
subplot(plotHandles.subplotHandle)
for i = 1:1:swarmModel.N
    switch swarmModel.communicationTopology
        case 'centralized'
            xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
        case 'allToAll'
            xk = [ swarmState{i}.x(1); swarmState{i}.x(2); swarmState{i}.x(3); swarmState{i}.x(4) ];
    end
    xc = xcnom + xk(1);
    yc = ycnom + xk(2);
    set(plotHandles.figh_sensingRadius(i),'XData',xc,'YData',yc);
end

axis equal;
% xlim([trueWorld.minX trueWorld.maxX]);
% ylim([trueWorld.minY trueWorld.maxY]);
if ( runParams.movie.useBackgroundImg )
%     xlim([trueWorld.minX-runParams.movie.plotBuffer trueWorld.maxX+runParams.movie.plotBuffer]);
%     ylim([trueWorld.minY-runParams.movie.plotBuffer trueWorld.maxY+runParams.movie.plotBuffer]);
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
else
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
end
end
