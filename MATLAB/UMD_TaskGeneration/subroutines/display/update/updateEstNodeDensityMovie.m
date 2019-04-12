function plotHandles = updateEstNodeDensityMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
plot(swarmState.t, swarmWorld.nodeDensityEstimate,'ko','linewidth',2);

end