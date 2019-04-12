function plotHandles = updateTargetViewsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
plot(swarmState.t, swarmWorld.numViews,'ro','linewidth',2);
hold on;
%ylim([0 2*swarmModel.cumlLRthresh])
xlim([0 runParams.T])
end