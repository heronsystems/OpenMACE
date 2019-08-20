function plotHandles  = updateLikelihoodWptsMovie( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)

subplot(plotHandles.subplotHandle)
% update sampling priority
set(plotHandles.figh_subplot2,'CData', log(swarmWorld.samplingPriority) ); 
for i = 1:1:length(swarmWorld.voronoiCells)
    ind = swarmWorld.voronoiCells{i};
    ind = [ind ind(1)];
    set(plotHandles.figh_voronoiCells(i),'XData',swarmWorld.voronoiVertices(ind,1),'YData',swarmWorld.voronoiVertices(ind,2));
end
set(plotHandles.figh_voronoiCenters,'XData',swarmWorld.cellCenterOfMass(:,1), 'YData',swarmWorld.cellCenterOfMass(:,2));

% 
for i = 1:1:swarmModel.N
    set(plotHandles.figh_voronoiCenters,'XData',swarmWorld.cellCenterOfMass(:,1), 'YData',swarmWorld.cellCenterOfMass(:,2));
    for j = 1:1:size(swarmState.wptList,2)
        ind = swarmState.wptList(i,j);
        bundleX(j) = swarmWorld.cellCenterOfMass(ind,1);
        bundleY(j) = swarmWorld.cellCenterOfMass(ind,2);        
    end
    set(plotHandles.figh_bundle(i),'XData',bundleX,'YData',bundleY);
end

end
