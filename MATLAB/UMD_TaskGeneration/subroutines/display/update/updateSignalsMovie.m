function plotHandles = updateSignalsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
hold off;
if (isfield(swarmWorld,'signals'))
    plot(swarmWorld.signals,'ko-','linewidth',2)
    hold on;
    plot(ones(size(swarmWorld.signals))*swarmModel.mZ,'r--','linewidth',2)
    ylim([-3 swarmModel.mZ+3])
    set(gca,'FontSize',14)
    xlabel('Index (Node in View)')
    ylabel('Signal Strength')
    grid on;
    hold on;
end
end