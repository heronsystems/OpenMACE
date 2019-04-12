function plotHandles = initPriorQMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

plotHandles.figh_subplot2 = imagesc(trueWorld.xcp,trueWorld.ycp,swarmWorld.U);
title('Prob \{ Cell = Unoccupied\}')
set(gca,'YDir','normal')
colorbar;
caxis([0 1])
set(gca,'FontSize',14)
xlabel('X (m)')
ylabel('Y (m)')
axis equal;
axis tight;
hold off;

end