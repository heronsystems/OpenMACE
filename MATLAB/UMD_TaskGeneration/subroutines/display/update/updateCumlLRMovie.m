function plotHandles = updateCumlLRMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

if (isfield(swarmWorld,'cumlLR'))
plot(swarmState.t, log(swarmWorld.cumlLR),'ro-','linewidth',2);
hold on;
plot(swarmState.t, log(swarmModel.cumlLRthresh),'k+','linewidth',2);
plotHandles.figh_subplot2 = plot(swarmState.t, log(swarmModel.LReqbm),'b+','linewidth',1);
set(gca,'FontSize',14)
xlabel('Time (sec)')
ylabel('Integrated Log Likelihood')
hold on;
ylim([-4 2*log(swarmModel.cumlLRthresh)])
xlim([0 runParams.T])
grid on;
end

end