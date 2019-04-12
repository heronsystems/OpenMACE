function plotHandles  = updateGroundTruthMovie( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)

%
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);

% loop through updates
subplot(plotHandles.subplotHandle)
for i = 1:1:swarmModel.N
    switch swarmModel.communicationTopology
        case 'centralized'
            xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
        case 'allToAll'
            xk = [ swarmState{i}.x(1); swarmState{i}.x(2); swarmState{i}.x(3); swarmState{i}.x(4) ];
    end
    xc = xcnom + xk(1);
    yc = ycnom + xk(2);
    set(plotHandles.figh_sensingRadius(i),'XData',xc,'YData',yc);
end
% % update target locations
% for i = 1:1:targetModel.M
%     if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
%         curNode = targetState.x(4*i-3);
%     elseif ( strcmp(targetModel.type, 'constantSpeedRandomWalk') || strcmp(targetModel.type, 'constantSpeedRandomWalkGenerative') )
%         curNode = targetState.x(2*i-1);
%     end
%     targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
%     set(plotHandles.figh_targetLoc(i),'XData',targXY(1),'YData',targXY(2));
% end
switch swarmModel.communicationTopology
    case 'centralized'
        %title(['Occupancy Graph, time = ' num2str(swarmWorld.time)]);
    otherwise
        title(['Occupancy Graph, time = ' num2str(swarmWorld{1}.time)]);
end
end
