function plotHandles  = initLikelihoodTargetStateExploredMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
set(gcf,'color','white')
%set(gca, 'color', [0.8 0.8 0.8])
plotHandles.p1 = plot(swarmWorld.targetStateSpaceGraph);
set(gca,'FontSize',14)
%title('Target State Space')
axis equal;
axis off;
p1.NodeCData = swarmWorld.log_likelihood;
p1.MarkerSize = 6;
%colorbar;
caxis([0 1]);

end