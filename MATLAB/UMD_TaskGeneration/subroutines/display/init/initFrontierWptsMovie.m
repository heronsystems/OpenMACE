function plotHandles  = initFrontierWptsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)

switch swarmModel.communicationTopology
    case 'allToAll'
        cellStateMat = swarmWorld{1}.cellStateMat;
        plotHandles.figh_subplot2 = imagesc(trueWorld.xcp,trueWorld.ycp,cellStateMat,[0 2]);
        title('Frontier (agent 1)')
        set(gca,'YDir','normal')
        % colorbar;
        set(gca,'FontSize',16)
        xlabel('X (m)')
        ylabel('Y (m)')
        
        hold on;
        % plot frontier nodes
        frontier = [trueWorld.nodeX((swarmWorld{1}.frontierIndex)) trueWorld.nodeY((swarmWorld{1}.frontierIndex))];
        if ~isempty(frontier)
            plotHandles.figh_frontierNodes = scatter(frontier(:,1),frontier(:,2),30,'r','s','linewidth',1,'MarkerFaceColor','flat');
        end
        
        % plot subblob centroids
        if ( strcmp(swarmModel.mapping.method,'frontierAndBlob') )
            plotHandles.figh_sublobCentroid = scatter(swarmWorld{1}.subblobCentroid(:,1),swarmWorld{1}.subblobCentroid(:,2),30,'k','o','linewidth',1,'MarkerFaceColor','flat');
        end
        
        for i=1:swarmModel.N
            xk = [ swarmState{i}.x(1); swarmState{i}.x(2); swarmState{i}.x(3); swarmState{i}.x(4) ];
            % add agent number
            plotHandles.figh_agentText(i) = text(xk(1),xk(2),num2str(i),'Color','white','FontSize',15);
            plotHandles.figh_agentNode(i) = plot([xk(1) swarmState{i}.xd],[xk(2) swarmState{i}.yd],'Color',[1 0 1],'LineWidth',2);
            
            % plot task bundle
            taskList = swarmWorld{i}.assignedBundle;
            
            if i == 1
                if ~isempty(taskList)
                    plotHandles.figh_taskBundle(i) = plot(taskList(:,1)',taskList(:,2)','r-','Marker','o','MarkerSize',10);
                else
                    plotHandles.figh_taskBundle(i) = plot([1],[1],'r-','Marker','o','MarkerSize',10);
                    set(plotHandles.figh_taskBundle(i),'Visible','off');
                end
            elseif i==2
                if ~isempty(taskList)
                    plotHandles.figh_taskBundle(i) = plot(taskList(:,1)',taskList(:,2)','g-','Marker','o','MarkerSize',10);
                else
                    plotHandles.figh_taskBundle(i) = plot([1],[1],'r-','Marker','o','MarkerSize',10);
                    set(plotHandles.figh_taskBundle(i),'Visible','off');
                end
            end
        end
        
        % plot current tasks (this one should come into the above for-loop)
        plotHandles.figh_currentTaskList = plot(swarmWorld{1}.currentTaskList(:,1),swarmWorld{1}.currentTaskList(:,2),'go','MarkerSize',4,'MarkerFaceColor','g');
                
        axis equal;
        xlim([trueWorld.minX-5 trueWorld.maxX+5]);
        ylim([trueWorld.minY-5 trueWorld.maxY+5]);
    case 'centralized'
        cellStateMat = swarmWorld.cellStateMat;
        plotHandles.figh_subplot2 = imagesc(trueWorld.xcp,trueWorld.ycp,cellStateMat,[0 2]);
        title('Frontier')
        set(gca,'YDir','normal')
        % colorbar;
        set(gca,'FontSize',16)
        xlabel('X (m)')
        ylabel('Y (m)')
        
        hold on;
        % plot frontier nodes
        frontier = [trueWorld.nodeX((swarmWorld.frontierIndex)) trueWorld.nodeY((swarmWorld.frontierIndex))];
        if ~isempty(frontier)
            plotHandles.figh_frontierNodes = scatter(frontier(:,1),frontier(:,2),30,'r','s','linewidth',1,'MarkerFaceColor','flat');
        end
        
        % plot subblob centroids
        if ( strcmp(swarmModel.mapping.method,'frontierAndBlob') )
            plotHandles.figh_sublobCentroid = scatter(swarmWorld.subblobCentroid(:,1),swarmWorld.subblobCentroid(:,2),30,'k','o','linewidth',1,'MarkerFaceColor','flat');
        end
        
        for i=1:swarmModel.N
            xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
            % add agent number
            plotHandles.figh_agentText(i) = text(xk(1),xk(2),num2str(i),'Color','white','FontSize',15);
            plotHandles.figh_agentNode(i) = plot([xk(1) swarmState.xd(i)],[xk(2) swarmState.yd(i)],'g');
        end
        axis equal;
        xlim([trueWorld.minX trueWorld.maxX]);
        ylim([trueWorld.minY trueWorld.maxY]);        
end
end