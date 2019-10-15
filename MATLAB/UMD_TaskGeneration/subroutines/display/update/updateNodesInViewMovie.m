function plotHandles = updateNodesInViewMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
plotHandles.figh_subplot2 = imagesc(trueWorld.xcp,trueWorld.ycp,swarmWorld.mutualInfoSurface);
V = swarmWorld.nodesInView;
if ( ~isempty(swarmWorld.exploredGraph.Nodes) )
    % trueGraphIndex = swarmWorld.exploredGraph.Nodes.trueGraphIndex;
    hold on;
    cmapTemp=gray(100);
    for i = 1:1:length( swarmWorld.nodesInView )
        %nodex = trueWorld.G_env.Nodes.x( trueGraphIndex(V(i)));
        %nodey = trueWorld.G_env.Nodes.y( trueGraphIndex(V(i)));
        bx = swarmWorld.exploredGraph.Nodes.bx( V(i) );
        by = swarmWorld.exploredGraph.Nodes.by( V(i) );
        nodex = trueWorld.xcp(bx);
        nodey = trueWorld.ycp(by);
        minVal = -3;
        maxVal = swarmModel.m + 3;
        interpVal = (swarmWorld.signals(i)-minVal) / (maxVal - minVal);
        colorInd = min(100,max( round(interpVal*100),0) + 1);
        plot(nodex,nodey,'ks','MarkerFaceColor',cmapTemp(colorInd,:))
        hold on;
    end
end
axis equal;
xlim([trueWorld.minX trueWorld.maxX])
ylim([trueWorld.minY trueWorld.maxY])
colorbar; 

% update target locations
for i = 1:1:targetModel.M
    if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
        curNode = targetState.x(4*i-3);
    elseif ( strcmp(targetModel.type, 'constantSpeedRandomWalk') || strcmp(targetModel.type, 'constantSpeedRandomWalkGenerative'))
        curNode = targetState.x(2*i-1);
    end
    targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
    set(plotHandles.figh_targetLoc(i),'XData',targXY(1),'YData',targXY(2));
end
% movie
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);
% plot sensing radius
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

if ( any( swarmWorld.cumlLR > swarmModel.cumlLRthresh ) )
    disp('Plotting most likely target location');
    [maxVal, maxInd] = max(swarmWorld.env_probPresent); %log_likelihood_env);
    % get corresponding node
    fprintf('Environment node %d has log likelihood %3.3f and probability %3.3f \n',maxInd, maxVal, swarmWorld.env_probPresent(maxInd));
    %trueGraphIndexMaxProb = swarmWorld.targetStateSpaceGraph.Nodes.curNode(maxInd);
    trueGraphIndexMaxProb = swarmWorld.exploredGraph.Nodes.trueGraphIndex(maxInd);
    % get nodeXY
    plot(trueWorld.nodeX(trueGraphIndexMaxProb), trueWorld.nodeY(trueGraphIndexMaxProb), 'mx' , 'MarkerSize', 14, 'linewidth',2);
end