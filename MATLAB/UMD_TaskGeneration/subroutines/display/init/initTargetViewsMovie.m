function plotHandles = initTargetViewsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

plotHandles.figh_subplot2 = plot(swarmState.t, swarmWorld.numViews,'ro','linewidth',2);
hold on;
%plotHandles.figh_subplot2 = plot(swarmState.t, swarmModel.cumlLRthresh,'k+','linewidth',2);
set(gca,'FontSize',14)
xlabel('Time (sec)')
ylabel('Target Views')
hold on;
%ylim([0 2*swarmModel.cumlLRthresh])
xlim([0 runParams.T])
grid on;
%axis square;
end