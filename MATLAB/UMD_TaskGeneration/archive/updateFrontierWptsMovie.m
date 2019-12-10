function plotHandles  = updateFrontierWptsMovie( swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
switch swarmModel.communicationTopology
    case 'allToAll'
        subplot(plotHandles.subplotHandle)
                
        % the following for-loop removes outdated agent-target line
        for i = 1:swarmModel.N
            set(plotHandles.figh_agentNode(i),'Visible','off');
            set(plotHandles.figh_taskBundle(i),'Visible','off');
        end
        set(plotHandles.figh_currentTaskList,'Visible','off');
        
        % update explored environment
        set(plotHandles.figh_subplot2,'CData', swarmWorld{1}.cellStateMat);
        
        % update frontier nodes
        frontier = [trueWorld.nodeX((swarmWorld{1}.frontierIndex)) trueWorld.nodeY((swarmWorld{1}.frontierIndex))];
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
            % plot agent current following task
            plotHandles.figh_agentNode(i) = plot([xk(1) swarmState{i}.xd],[xk(2) swarmState{i}.yd],'Color',[1 0 1],'LineWidth',2);
            
            % plot task bundle
            taskList = swarmWorld{i}.assignedBundle;
            if ~isempty(taskList)
                if i == 1
                    plotHandles.figh_taskBundle(i) = plot(taskList(:,1)',taskList(:,2)','r-','Marker','o','MarkerSize',10);
                elseif i==2
                    plotHandles.figh_taskBundle(i) = plot(taskList(:,1)',taskList(:,2)','g-','Marker','o','MarkerSize',10);
                end
            end
        end
        
        % plot current tasks (this one should come into the above for-loop)
        plotHandles.figh_currentTaskList = plot(swarmWorld{1}.currentTaskList(:,1),swarmWorld{1}.currentTaskList(:,2),'go','MarkerSize',4,'MarkerFaceColor','g');
               
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
        if sum(strcmp(fieldnames(plotHandles),'figh_frontierNodes')) % to replace this command: exist('plotHandles.figh_frontierNodes')  
            % if the plothandle exists
            set(plotHandles.figh_frontierNodes,'XData',frontier(:,1),'YData',frontier(:,2));
        elseif ~isempty(frontier) && ~sum(strcmp(fieldnames(plotHandles),'figh_frontierNodes'))
            % if frontier is nonempty and the plot handle does not exist
            plotHandles.figh_frontierNodes = scatter(frontier(:,1),frontier(:,2),30,'r','s','linewidth',1,'MarkerFaceColor','flat');
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

