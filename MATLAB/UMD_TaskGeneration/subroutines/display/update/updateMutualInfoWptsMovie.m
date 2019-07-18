function plotHandles  = updateMutualInfoWptsMovie( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)

subplot(plotHandles.subplotHandle)
% update sampling priority
set(plotHandles.figh_subplot2,'CData', swarmWorld.mutualInfoSurface );

for i = 1:1:length(swarmWorld.voronoiCells)
    ind = swarmWorld.voronoiCells{i};
    ind = [ind ind(1)];
    set(plotHandles.figh_voronoiCells(i),'XData',swarmWorld.voronoiVertices(ind,1),'YData',swarmWorld.voronoiVertices(ind,2));
end
set(plotHandles.figh_voronoiCenters,'XData',swarmWorld.cellCenterOfMass(:,1), 'YData',swarmWorld.cellCenterOfMass(:,2));

%
for i = 1:1:swarmModel.N
    set(plotHandles.figh_voronoiCenters,'XData',swarmWorld.cellCenterOfMass(:,1), 'YData',swarmWorld.cellCenterOfMass(:,2));
    for j = 1:1:size(swarmState.wptList,2)%-2
        ind = swarmState.wptList(i,j);
        bundleX(i,j) = swarmWorld.cellCenterOfMass(ind,1);
        bundleY(i,j) = swarmWorld.cellCenterOfMass(ind,2);
    end
    % add agent current poosition
    xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
    %plot([xk(1) bundleX],[xk(2) bundleY],'mo-','linewidth',2,'MarkerFaceColor','w');
    set(plotHandles.figh_bundle(i),'XData',bundleX(i,:),'YData',bundleY(i,:));
end
% visited wpts
for i = 1:1:swarmModel.N
    if (swarmState.wptIndex(i) > 1)
        %
        if ( isfield(plotHandles,'figh_visitedWpt') )
        if ( length(isgraphics(plotHandles.figh_visitedWpt)) >= i )
        if ( isgraphics(plotHandles.figh_visitedWpt(i)) )
            set(plotHandles.figh_visitedWpt(i),'XData',bundleX(i,1:swarmState.wptIndex(i)-1),'YData',bundleY(i,1:swarmState.wptIndex(i)-1));
        end
        end
        else
            plotHandles.figh_visitedWpt(i) = plot(bundleX(i,1:swarmState.wptIndex(i)-1),bundleY(i,1:swarmState.wptIndex(i)-1),'mo','linewidth',2,'MarkerFaceColor','m');
        end    
    else
        if ( isfield(plotHandles,'figh_visitedWpt') )
        if ( length(isgraphics(plotHandles.figh_visitedWpt)) >= i )
        if ( isgraphics(plotHandles.figh_visitedWpt(i)) )
            set(plotHandles.figh_visitedWpt(i),'XData',[],'YData',[]);
        end
        end
        end        
    end
        
        
end

% plot Path
for i = 1:1:swarmModel.N
    pathX = [];
    pathY = [];
    for j = 1:1:size(swarmState.wptList,2);
        wpt1 = swarmWorld.initialNodes{j}(i);
        wpt2 = swarmState.wptList(i,j);
        % convert to index in initialNodes
        ind1 = 1;
        % convert to index in terminalNodes
        for k = 1:1:length(swarmWorld.targetNodes{j});
            if (wpt2 == swarmWorld.targetNodes{j}(k));
                ind2 = k;
            end
        end
        %
        if ( ~isempty(swarmWorld.pathHistory{i,j,ind2}) )
            pathX = [pathX; swarmWorld.pathHistory{i,j,ind2}(:,1)];
            pathY = [pathY; swarmWorld.pathHistory{i,j,ind2}(:,2)];
        end
    end
    set(plotHandles.figh_path(i),'XData',pathX,'YData',pathY);
end


%
numPts = 20;
[xcnom, ycnom] = generateCircle(0, 0, swarmModel.Rsense, numPts);

% plot sensing radius
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

axis equal;
% xlim([trueWorld.minX trueWorld.maxX]);
% ylim([trueWorld.minY trueWorld.maxY]);
if ( runParams.movie.useBackgroundImg )
    %     xlim([trueWorld.minX-runParams.movie.plotBuffer trueWorld.maxX+runParams.movie.plotBuffer]);
    %     ylim([trueWorld.minY-runParams.movie.plotBuffer trueWorld.maxY+runParams.movie.plotBuffer]);
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
else
    xlim([trueWorld.minX trueWorld.maxX]);
    ylim([trueWorld.minY trueWorld.maxY]);
end
end
