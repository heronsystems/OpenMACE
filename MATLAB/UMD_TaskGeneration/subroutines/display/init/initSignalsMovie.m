function plotHandles = initSignalsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

hold off;
if (isfield(swarmWorld,'signals'))
plot(swarmWorld.signals,'ko-','linewidth',2)
ylim([-3 swarmModel.mZ+3])
set(gca,'FontSize',14)
xlabel('Index')
ylabel('Signal Strength')
grid on;
hold on;
end



end