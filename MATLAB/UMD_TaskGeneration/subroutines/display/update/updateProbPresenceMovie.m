function plotHandles = updateProbPresenceMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
hold off;
plot(exp(swarmWorld.log_likelihood_env),'linewidth',2)
set(gca,'YDir','normal')
%ylim([0 1])
xlim([1 trueWorld.numNodes])
set(gca,'FontSize',14)
xlabel('Node ID')
ylabel('Likelihood Ratio')
hold on;

end