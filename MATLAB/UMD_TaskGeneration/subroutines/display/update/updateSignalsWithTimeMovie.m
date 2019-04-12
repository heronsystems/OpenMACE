function plotHandles = updateSignalsWithTimeMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
for i = 1:1:length(swarmWorld.signals)
    plot(swarmState.t, swarmWorld.signals(i),'k.','linewidth',1)
    hold on;
end
if ( ~isempty(swarmWorld.signals) )
plot(swarmState.t, max(swarmWorld.signals),'ro-','linewidth',2);
hold on;
plot(swarmState.t, min(swarmWorld.signals),'bo-','linewidth',2);
end
% hold on;
% plot(swarmState.t, min(swarmWorld.signals),'bo-','linewidth',2);
plot(swarmState.t, swarmModel.m,'k+','linewidth',2);
%plot(swarmWorld.signals,'ko-','linewidth',2)
ylim([-3 swarmModel.m+3]);
xlim([0 runParams.T])
set(gca,'FontSize',14)
grid on;
hold on;
end