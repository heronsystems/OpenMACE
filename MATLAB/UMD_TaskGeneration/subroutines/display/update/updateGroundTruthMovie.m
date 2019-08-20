function plotHandles  = updateGroundTruthMovie( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)

%
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);

% loop through updates
subplot(plotHandles.subplotHandle)
for i = 1:1:swarmModel.N
    xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
    xc = xcnom + xk(1);
    yc = ycnom + xk(2);
    set(plotHandles.figh_sensingRadius(i),'XData',xc,'YData',yc);
end
% % update target locations
% for i = 1:1:targetModel.M
% curNode = targetState.x(2*i-1);
%     targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
%     set(plotHandles.figh_targetLoc(i),'XData',targXY(1),'YData',targXY(2));
% end
title(['Occupancy Graph, time = ' num2str(swarmWorld.time)]);

end
