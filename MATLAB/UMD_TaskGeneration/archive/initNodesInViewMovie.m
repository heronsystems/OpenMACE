function plotHandles = initNodesInViewMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

plotHandles.figh_subplot2 = imagesc(trueWorld.xcp,trueWorld.ycp, swarmWorld.mutualInfoSurface);
title('Mutual Information')
set(gca,'YDir','normal')
caxis([0 1])
set(gca,'FontSize',14)
xlabel('X (m)')
ylabel('Y (m)')
axis equal;
axis tight;
hold on;
colorbar;

% plot target locations
for i = 1:1:targetModel.M
    if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
        curNode = targetState.x(4*i-3);
    elseif ( strcmp(targetModel.type, 'constantSpeedRandomWalk') || strcmp(targetModel.type, 'constantSpeedRandomWalkGenerative') )
        curNode = targetState.x(2*i-1);
    end
    targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
    plotHandles.figh_targetLoc(i) = plot(targXY(1), targXY(2), 'r+','linewidth',2);
end

% movie
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);
% plot sensing radius
for i = 1:1:swarmModel.N
    xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
    xc = xcnom + xk(1);
    yc = ycnom + xk(2);
    plotHandles.figh_sensingRadius(i) = plot(xc,yc,'k-');
end

axis equal;
xlim([trueWorld.minX trueWorld.maxX])
ylim([trueWorld.minY trueWorld.maxY])

end