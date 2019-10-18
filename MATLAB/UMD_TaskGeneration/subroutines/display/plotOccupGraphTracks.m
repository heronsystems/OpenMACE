function plotOccupGraphTracks(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )

numRows = size(swarmWorldHist{1}.U,1);
numCols = size(swarmWorldHist{1}.U,2);
% vectorize cell array data
for i = 1:1:length(swarmWorldHist)
    t(i) = swarmWorldHist{i}.time;
    %nodeDensityEstimate(i) = swarmWorldHist{i}.nodeDensityEstimate;
    % compute entropy of entire search grid
    entropyOfCells = swarmWorldHist{i}.entropyMat;
    totalEntropy(i) = swarmWorldHist{i}.totalEntropy;
    % compute the percentage of discovered nodes
    discoveredNodePercentage(i) = 100*numnodes(swarmWorldHist{i}.exploredGraph)/numnodes(trueWorld.G_env);
    % swarm state (x,y)
    for j = 1:1:swarmModel.N
        swarmState = swarmStateHist{i};
        xk = [ swarmState.x(4*j-3); swarmState.x(4*j-2); swarmState.x(4*j-1); swarmState.x(4*j) ];
        swarmAgentX(i,j) = xk(1);
        swarmAgentY(i,j) = xk(2);
        swarmAgentXd(i,j) = swarmState.xd(j);
        swarmAgentYd(i,j) = swarmState.yd(j);
        
    end
end

numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);

figh3 = figure;
figh2 = figure;
figh = figure;
set(gcf,'color','white')
axis equal;

s = swarmModel.samplesPerTask;
for i = 1:s:length(swarmWorldHist)
    % plot path of agents
    clf(figh);
    figure(figh);
    set(gca,'YDir','Normal')
    hold off;
    axis equal;
    axis tight;
    swarmWorld = swarmWorldHist{i}'
    plot(trueWorld.nodeX, trueWorld.nodeY,'o', 'color', [0.8 0.8 0.8])
    if ( ~isempty(swarmWorld.exploredGraph.Nodes) )
        xData = trueWorld.xcp( swarmWorld.exploredGraph.Nodes.bx );
        yData = trueWorld.ycp( swarmWorld.exploredGraph.Nodes.by );
        hold on;
        %plot(swarmWorld.exploredGraph,'XData',xData,'YData',yData,'NodeLabel',[],'MarkerSize',6);
        plotHandles.p1 = plot(swarmWorld.exploredGraph,'XData',xData,'YData',yData,'NodeLabel',[],'NodeColor','b');
        plotHandles.p1.NodeCData = swarmWorld.log_likelihood_env;
        plotHandles.p1.MarkerSize = 6;
        
    end
    
    % plot target locations
    targetState = targetStateHist{i};
    for j = 1:1:targetModel.M
        curNode = targetState.x(2*j-1);
        targXY = [trueWorld.nodeX(curNode) trueWorld.nodeY(curNode)];
        plotHandles.figh_targetLoc(j) = plot(targXY(1), targXY(2), 'rx','linewidth',1,'MarkerSize',8);
    end
    
    colormap(gca,'parula')
    cb1 = colorbar;
    cb1.Label.String = 'Log Likelihood';
    cb1.Label.Rotation = 90; % to rotate the text
    cb1.FontSize = 16;
    caxis([-14 0]);
    
    
    
    swarmState = swarmStateHist{i};
    % plot sensing radius
    for j = 1:1:swarmModel.N
        xk = [ swarmState.x(4*j-3); swarmState.x(4*j-2); swarmState.x(4*j-1); swarmState.x(4*j) ];
        xc = xcnom + xk(1);
        yc = ycnom + xk(2);
        plot(xc,yc,'m-','linewidth',2);
        hold on;
    end
    
    
    for j = 1:1:swarmModel.N
        x = swarmAgentX(1:i,j);
        y = swarmAgentY(1:i,j);
        plot(x,y,'-','linewidth',2);
        hold on;
        xlabel('X (m)')
        ylabel('Y (m)')
        %grid on;
        set(gca,'FontSize',16,'FontName','Arial');
    end
    axis equal;
    xticks([0,50,100,150])
    yticks([0,50,100,150])
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
    drawnow;
    
    
    clf(figh2);
    figure(figh2);
    set(gcf,'color','white')
    p2 = plot(swarmWorld.targetStateSpaceGraph);
    set(gca,'FontSize',16,'FontName','Arial');
    axis equal;
    axis off;
    if ( ~isempty(swarmWorld.log_likelihood) )
        p2.NodeCData = swarmWorld.log_likelihood;
        p2.MarkerSize = 6;
    end
    cb2 = colorbar('eastoutside');
    cb2.Label.String = 'Log Likelihood';
    cb2.FontSize = 16;
    %pos2 = get(cb2,'Position');
    %cb2.Label.Position = [pos2(1)/2 pos2(2)+1]; % to change its position
    cb2.Label.Rotation = 90; % to rotate the text
    caxis([-14 0]);
    drawnow;
    
    
    clf(figh3);
    figure(figh3);
    
    imagesc(trueWorld.xcp,trueWorld.ycp, swarmWorld.samplingPriority );
    set(gca,'YDir','normal')
    %title('Sampling Priority Surface')
    cb3 = colorbar;
    cb3.Label.String = 'Mutual Information (bits)';
    %pos3 = get(cb3,'Position');
    %cb3.Label.Position = [pos3(1)/2 pos3(2)+1]; % to change its position
    cb3.Label.Rotation = 90; % to rotate the text
    cb3.FontSize = 16;
    %caxis([0 1])
    set(gca,'FontSize',16,'FontName','Arial');
    xlabel('X (m)')
    ylabel('Y (m)')
    hold on;
    for i = 1:1:length(swarmWorld.voronoiCells)
        ind = swarmWorld.voronoiCells{i};
        ind = [ind ind(1)];
        plotHandles.figh_voronoiCells(i) = plot(swarmWorld.voronoiVertices(ind,1),swarmWorld.voronoiVertices(ind,2),'Color',[1 1 1]*0.8,'linewidth',1);
        hold on;
    end
    plot(swarmWorld.cellCenterOfMass(:,1), swarmWorld.cellCenterOfMass(:,2),'+','Color',[1 1 1]*0.8,'linewidth',1);
    
    % plot bundles
    for k = 1:1:swarmModel.N
        %set(plotHandles.figh_voronoiCenters,'XData',swarmWorld.cellCenterOfMass(:,1), 'YData',swarmWorld.cellCenterOfMass(:,2));
        for j = 1:1:size(swarmState.wptList,2)
            ind = swarmState.wptList(k,j);
            bundleX(j) = swarmWorld.cellCenterOfMass(ind,1);
            bundleY(j) = swarmWorld.cellCenterOfMass(ind,2);
        end
        % add agent current poosition
        xk = [ swarmState.x(4*k-3); swarmState.x(4*k-2); swarmState.x(4*k-1); swarmState.x(4*k) ];
        plot([xk(1) bundleX],[xk(2) bundleY],'mo-','linewidth',2,'MarkerFaceColor','m');
        
    end
    
    % plot sensing radius
    for j = 1:1:swarmModel.N
        xk = [ swarmState.x(4*j-3); swarmState.x(4*j-2); swarmState.x(4*j-1); swarmState.x(4*j) ];
        xc = xcnom + xk(1);
        yc = ycnom + xk(2);
        plot(xc,yc,'m-','linewidth',2);
        hold on;
    end
    
    axis equal;
    set(gcf,'Color','w')
    xticks([0,50,100,150])
    yticks([0,50,100,150])
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
    drawnow;
    
    
    
    pause;
end



end