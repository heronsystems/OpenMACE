function plotHandles  = updateLikelihoodTargetStateExploredMovie( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)

subplot(plotHandles.subplotHandle)

set(gcf,'color','white')
set(gca, 'color', [0.8 0.8 0.8])
if ( ~isempty(swarmWorld.targetStateSpaceGraph.Nodes) )
    plotHandles.p1 = plot(swarmWorld.targetStateSpaceGraph);
    plotHandles.p1.NodeCData = swarmWorld.log_likelihood;
    %plotHandles.p1.NodeCData = swarmWorld.tss_probPresent; 
    plotHandles.p1.MarkerSize = 6;
end
set(gca,'FontSize',14)
title('Target State Graph')
axis equal;
axis square;
axis off;
%colorbar;
caxis([-10 10]);

end
