function plotHandles = initKrigingInterpMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

plotHandles.figh_subplot2 = imagesc(trueWorld.xcp(1,:),trueWorld.ycp(:,1)',swarmWorld.forecastMat);
title('Krig. Intrp.')
set(gca,'YDir','normal')
% colorbar;
set(gca,'FontSize',14)
xlabel('X (m)')
ylabel('Y (m)')
axis equal;
axis tight;

end