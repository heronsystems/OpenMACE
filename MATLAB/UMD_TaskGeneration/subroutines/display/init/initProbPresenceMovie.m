function plotHandles = initProbPresenceMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
plot(exp(swarmWorld.log_likelihood_env),'linewidth',2)
hold on;
set(gca,'YDir','normal')
%ylim([0 1])
xlim([1 trueWorld.numNodes])
set(gca,'FontSize',14)
xlabel('Node ID')
ylabel('Prob Presence')
hold on;




end