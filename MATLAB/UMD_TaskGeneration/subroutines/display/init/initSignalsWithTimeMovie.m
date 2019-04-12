function plotHandles = initSignalsWithTimeMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

%plotHandles.figh_subplot2 = plot(swarmState.t, max(swarmWorld.signals),'ro-','linewidth',2);
for i = 1:1:length(swarmWorld.signals)
    plot(swarmState.t, swarmWorld.signals(i),'k.','linewidth',1)
    hold on;
end
plot(swarmState.t, max(swarmWorld.signals),'ro-','linewidth',2);
hold on;
plot(swarmState.t, min(swarmWorld.signals),'bo-','linewidth',2);
plot(swarmState.t, swarmModel.m,'k+','linewidth',2);
ylim([-3 swarmModel.m+3]);
xlim([0 runParams.T])
set(gca,'FontSize',14)
xlabel('Time (sec.)')
ylabel('Target Measurements')
grid on;
hold on;



end