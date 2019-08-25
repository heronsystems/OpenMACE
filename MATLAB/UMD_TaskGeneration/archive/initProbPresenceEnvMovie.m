function plotHandles = initProbPresenceMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

plotHandles.figh_subplot2 = imagesc(trueWorld.xcp(1,:),trueWorld.ycp(:,1)', swarmWorld.priorP);
title('Nodes In View')
set(gca,'YDir','normal')
caxis([0 1])
set(gca,'FontSize',14)
xlabel('X (m)')
ylabel('Y (m)')
axis equal;
axis tight;
hold on;

% plot target locations
for i = 1:1:targetModel.M
    if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
        curNode = targetState.x(4*i-3);
    elseif ( strcmp(targetModel.type, 'constantSpeedRandomWalk') )
        curNode = targetState.x(2*i-1);
    end
    targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
    plotHandles.figh_targetLoc(i) = plot(targXY(1), targXY(2), 'r+','linewidth',2);
end

end