function plotHandles = initCumlLRMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)


if (isfield(swarmWorld,'cumlLR'))
plotHandles.figh_subplot2 = plot(swarmState.t, log(swarmWorld.cumlLR),'ro-','linewidth',2);
hold on;
plotHandles.figh_subplot2 = plot(swarmState.t, log(swarmModel.cumlLRthresh),'k+','linewidth',1);
plotHandles.figh_subplot2 = plot(swarmState.t, log(swarmModel.LReqbm),'b+','linewidth',1);
set(gca,'FontSize',14)
xlabel('Time (sec)')
ylabel('Integrated Log Likelihood')
hold on;
ylim([-4 2*log(swarmModel.cumlLRthresh)])
xlim([0 runParams.T])
grid on;
end
%axis square;
end