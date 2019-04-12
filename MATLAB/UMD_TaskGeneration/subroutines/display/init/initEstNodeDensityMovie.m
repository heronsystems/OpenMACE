function plotHandles = initEstNodeDensityMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

plotHandles.figh_subplot2 = plot(swarmState.t, swarmWorld.nodeDensityEstimate,'ko','linewidth',2);
title('Explored Area Node Density')
set(gca,'YDir','normal')
set(gca,'FontSize',14)
xlabel('Time (sec)')
ylabel('Percent of Explored Cells (%)')
hold on;
ylim([0 0.5])
xlim([0 runParams.T])
grid on;
%axis square;
end