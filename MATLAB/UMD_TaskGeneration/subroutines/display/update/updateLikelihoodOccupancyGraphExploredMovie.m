function plotHandles  = updateLikelihoodOccupancyGraphExploredMovie( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)

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


%
% update target locations
for i = 1:1:targetModel.M
    curNode = targetState.x(2*i-1);
    targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
    set(plotHandles.figh_targetLoc(i),'XData',targXY(1),'YData',targXY(2),'ZData',1);
end

% update target state graph
if ( ~isempty(swarmWorld.exploredGraph.Nodes) )
    hold on;
    plotHandles.p1 = plot(swarmWorld.exploredGraph,'XData',swarmWorld.exploredGraph.Nodes.nodeX,'YData',swarmWorld.exploredGraph.Nodes.nodeY,'NodeLabel',[]);    
    plotHandles.p1.NodeCData = swarmWorld.log_likelihood_env;
    plotHandles.p1.MarkerSize = 6;
end
hold off;

% colorbar;
%caxis([0 1]);

end
