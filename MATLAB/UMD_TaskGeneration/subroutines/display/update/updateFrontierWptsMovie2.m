function plotHandles  = updateFrontierWptsMovie2( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)

switch swarmModel.communicationTopology
    case 'allToAll'
        subplot(plotHandles.subplotHandle)
        % the following for-loop removes outdated agent-target line
        for i = 1:swarmModel.N
            set(plotHandles.figh_agentNode(i),'Visible','off');
        end
        
        % update explored environment
        set(plotHandles.figh_subplot2,'CData', swarmWorld{2}.cellStateMat);

        % update frontier nodes
        frontier = [trueWorld.nodeX((swarmWorld{2}.frontierIndex)) trueWorld.nodeY((swarmWorld{2}.frontierIndex))];
        
        if sum(strcmp(fieldnames(plotHandles),'figh_frontierNodes')) % to replace this command: exist('plotHandles.figh_frontierNodes')  
            % if the plothandle exists
            set(plotHandles.figh_frontierNodes,'XData',frontier(:,1),'YData',frontier(:,2));
        elseif ~isempty(frontier) && ~sum(strcmp(fieldnames(plotHandles),'figh_frontierNodes'))
            % if frontier is nonempty and the plot handle does not exist
            plotHandles.figh_frontierNodes = scatter(frontier(:,1),frontier(:,2),30,'r','s','linewidth',1,'MarkerFaceColor','flat');
        end
        % update subblob centroids
        if ( strcmp(swarmModel.mapping.method,'frontierAndBlob') )
            set(plotHandles.figh_sublobCentroid,'XData',swarmWorld{1}.subblobCentroid(:,1),'YData',swarmWorld{1}.subblobCentroid(:,2));
        end
        % update agent text/assigned node
        for i=1:swarmModel.N
            xk = [ swarmState{i}.x(1); swarmState{i}.x(2); swarmState{i}.x(3); swarmState{i}.x(4) ];
            % add agent number
            set(plotHandles.figh_agentText(i),'Position',[xk(1) xk(2)]);
            plotHandles.figh_agentNode(i) = plot([xk(1) swarmState{i}.xd],[xk(2) swarmState{i}.yd],'g');
        end
        
    case 'centralized'
        subplot(plotHandles.subplotHandle)
        % the following for-loop removes outdated agent-target line
        for i = 1:swarmModel.N
            set(plotHandles.figh_agentNode(i),'Visible','off');
        end
        % update explored environment
        set(plotHandles.figh_subplot2,'CData', swarmWorld.cellStateMat);
        % update frontier nodes
        frontier = [trueWorld.nodeX((swarmWorld.frontierIndex)) trueWorld.nodeY((swarmWorld.frontierIndex))];
        if ~isempty(frontier)
        %     if ( exist('plotHandles.figh_frontierNodes') )
                set(plotHandles.figh_frontierNodes,'XData',frontier(:,1),'YData',frontier(:,2));
        %     else
        %        plotHandles.figh_frontierNodes = scatter(frontier(:,1),frontier(:,2),30,'r','s','linewidth',1,'MarkerFaceColor','flat');
        %     end
        end
        % update subblob centroids
        if ( strcmp(swarmModel.mapping.method,'frontierAndBlob') )
            set(plotHandles.figh_sublobCentroid,'XData',swarmWorld.subblobCentroid(:,1),'YData',swarmWorld.subblobCentroid(:,2));
        end
        % update agent text/assigned node
        for i=1:swarmModel.N
            xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
            % add agent number
            set(plotHandles.figh_agentText(i),'Position',[xk(1) xk(2)]);
            plotHandles.figh_agentNode(i) = plot([xk(1) swarmState.xd(i)],[xk(2) swarmState.yd(i)],'g');
        end
end
end

