function [swarmState, swarmWorld] = taskAllocation(tasks, swarmState, swarmModel, swarmWorld, trueWorld, simParams)
% This function should produce a desired waypoint or desired waypoint list
% for each agent
% taskManagment assigns the waypoints in the list
if strcmp(swarmModel.taskAllocation,'stepwiseHungarian') || strcmp(swarmModel.taskAllocation,'stepwiseHungarian_2ndOrder')
    if ( strcmp(swarmModel.taskGeneration,'mutualInfoWpts') || strcmp(swarmModel.taskGeneration,'likelihoodWpts') )
        if strcmp(swarmModel.neighborMethod,'VoronoiGraph') % this method generates neighbor nodes by the VoronoiGraph
            % do stepwise Hungarian to generate bundle
            
            tic;
            % 1. Generate the connectivity graph over the voronoi
            % cells (total time for 100 cells: 0.6s)
            VoronoiGraph = graph;
            VoronoiGraph = addnode(VoronoiGraph,size(swarmWorld.cellCenterOfMass,1));
            VoronoiGraph.Nodes.centerOfmass = swarmWorld.cellCenterOfMass;
            
            
            % turn the boundareis of each cell to the form of line
            % segments
            for kk = 1:size(swarmWorld.cellCenterOfMass,1)
                boundary{kk} = [swarmWorld.voronoiCells{kk}' [swarmWorld.voronoiCells{kk}(end);swarmWorld.voronoiCells{kk}(1:end-1)']];
            end
            
            for kk = 1:size(swarmWorld.cellCenterOfMass,1)-1
                for jj = 1:size(boundary{kk},1) % search for shared boudnaries to determine neighborhood
                    for ii = kk+1:size(swarmWorld.cellCenterOfMass,1) % search for this boundary in all other cells
                        if ismember(boundary{kk}(jj,:),boundary{ii},'rows') || ismember([boundary{kk}(jj,2) boundary{kk}(jj,1)],boundary{ii},'rows') % also need to consider the reversely ordered boundary
                            VoronoiGraph = addedge(VoronoiGraph,kk,ii);
                        end
                    end
                end
            end
            
            VoronoiGraph = simplify(VoronoiGraph);
            disp('Voronoi Graph')
            toc;
            
            % initial cells are given by the cells where agents
            % reside
            % first determine the closest four cell centers and see
            % if the agent is in one of them
            initialNodes = [];
            
            for kk = 1:swarmModel.N
                idx = knnsearch(swarmWorld.cellCenterOfMass,swarmState.x(4*(kk-1)+1:4*(kk-1)+2),'K',5); % find four nearest neighbors
                for ii = 1:5
                    VC = swarmWorld.voronoiVertices(swarmWorld.voronoiCells{idx(ii)},:);
                    initialNodeIndex = find(inpolygon(swarmState.x(4*(kk-1)+1),swarmState.x(4*(kk-1)+2),VC(:,1),VC(:,2)));
                    if initialNodeIndex ~= 0
                        initialNodes = [initialNodes; idx(ii)];
                        break;
                    end
                end
                if length(initialNodes)<kk % if no Voronoi cell contains the agent, just give the cell of closet centroid to the agent.
                    initialNodes = [initialNodes; idx(1)];
                end
            end
            
            q = swarmModel.bundleSize; % queue size
            bundleSteps = initialNodes;
            
            % load initial states
            state = zeros(swarmModel.N,4);
            for kk = 1:swarmModel.N
                state(kk,:) = swarmState.x(4*(kk-1)+1:4*(kk-1)+4);
            end
            
            visitedEdges = [-1 -1];
            
            infoSurfaceForAssignment = swarmWorld.mutualInfoSurface;
            % build the bundle
            for p = 1:q
                
                if strcmp(swarmModel.taskAllocation,'stepwiseHungarian')
                    % update the target nodes
                    targetNodes = [];
                    for kk = 1:swarmModel.N
                        targetNodes = union(targetNodes,neighbors(VoronoiGraph,initialNodes(kk)));
                    end
                elseif strcmp(swarmModel.taskAllocation,'stepwiseHungarian_2ndOrder')
                    % update the target nodes
                    targetNodes_1st = [];
                    for kk = 1:swarmModel.N
                        targetNodes_1st = union(targetNodes_1st,neighbors(VoronoiGraph,initialNodes(kk)));
                    end
                    targetNodes = targetNodes_1st;
                    for kk = length(targetNodes_1st)
                        targetNodes = union(targetNodes,neighbors(VoronoiGraph,targetNodes_1st(kk)));
                    end
                end
                % placeholder for likely locations where agents end
                terminalState = cell(length(initialNodes),length(targetNodes));
                
                % placeholder for utility
                utilityMatrix = zeros(length(initialNodes),length(targetNodes));
                % use adjacency matrix for connectivity
                adjacencyMatrix = adjacency(VoronoiGraph);
                
                if strcmp(swarmModel.taskAllocation,'stepwiseHungarian')
                    for kk = 1:length(initialNodes)
                        for jj = 1:length(targetNodes)
                            if adjacencyMatrix(initialNodes(kk),targetNodes(jj)) == 1 % if these two edges are connected
                                [temp1,temp2] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(targetNodes(jj),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state
                                utilityMatrix(kk,jj) = temp1;
                                terminalState{kk,jj} = temp2;
                            else
                                utilityMatrix(kk,jj) = -10; % if not connected, then the utitily is -10
                            end
                        end
                    end
                elseif strcmp(swarmModel.taskAllocation,'stepwiseHungarian_2ndOrder')
                    adjacencyMatrix_2ndOrder = adjacencyMatrix*adjacencyMatrix;
                    for kk = 1:length(initialNodes)
                        for jj = 1:length(targetNodes)
                            if adjacencyMatrix(initialNodes(kk),targetNodes(jj)) == 1 ||... % if these two edges are connected
                                    adjacencyMatrix_2ndOrder(initialNodes(kk),targetNodes(jj)) == 1 % if the nodes are connected by two edges (2nd order neighbor)
                                [temp1,temp2] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(targetNodes(jj),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state
                                utilityMatrix(kk,jj) = temp1;
                                terminalState{kk,jj} = temp2;
                            else
                                utilityMatrix(kk,jj) = -10; % if not connected, then the utitily is -10
                            end
                        end
                    end
                end
                
                % Hungarian
                assignmentIndex = munkres(-utilityMatrix);
                initialNodesNew = targetNodes(assignmentIndex);
                % add assignment to bundle
                bundleSteps = [bundleSteps initialNodesNew];
                
                for kk = 1:swarmModel.N
                    % update the infomation surface sequentially
                    [~,~,infoSurfaceForAssignment] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(initialNodesNew(kk),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state
                    % update initial states for the next step
                    state(kk,:) = terminalState{kk,assignmentIndex(kk)};
                end
                
                % remove the visited edges
                % VoronoiGraph = rmedge(VoronoiGraph,initialNodes,initialNodesNew);
                
                % update visited edges
                visitedEdges = union(visitedEdges,[initialNodes,initialNodesNew],'rows');
                
                % update the initialNodes
                initialNodes = initialNodesNew;
            end
            
            disp('bundle built')
            toc;
            disp('----')
            
            swarmState.wptList = bundleSteps(:,2:end);
            
            for kk = 1:swarmModel.N
                swarmState.xd(kk) = swarmWorld.cellCenterOfMass(swarmState.wptList(kk,1),1);
                swarmState.yd(kk) = swarmWorld.cellCenterOfMass(swarmState.wptList(kk,1),2);
                swarmState.wptIndex(kk) = 1;
            end
            
            %         % verification ===================
            %         figure;
            %         plot((VoronoiGraph),'XData',swarmWorld.cellCenterOfMass(:,1),'YData',swarmWorld.cellCenterOfMass(:,2))
            %         hold on
            %         for ii = 1:1:100
            %             ind = swarmWorld.voronoiCells{ii};
            %             ind = [ind ind(1)];
            %             plot(swarmWorld.voronoiVertices(ind,1),swarmWorld.voronoiVertices(ind,2),'r');
            %         end
            %
            %         hold on;
            %         plot(swarmWorld.cellCenterOfMass(swarmState.wptList(1,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(1,:),2),'k','LineWidth',2)
            %         plot(swarmWorld.cellCenterOfMass(swarmState.wptList(2,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(2,:),2),'b','LineWidth',2)
            %         plot(swarmWorld.cellCenterOfMass(swarmState.wptList(3,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(3,:),2),'g','LineWidth',2)
            %         plot(swarmWorld.cellCenterOfMass(swarmState.wptList(4,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(4,:),2),'c','LineWidth',2)
            %         % ================================
            %         5;
        elseif strcmp(swarmModel.neighborMethod,'knn') % this method generates neighbor nodes by the k nearest neighbors(knn)
            
            knnNumber = 5;
            % do stepwise Hungarian to generate bundle
            
            %             tic;
            %             % 1. Generate the connectivity graph over the voronoi
            %             % cells (total time for 100 cells: 0.6s)
            %             VoronoiGraph = graph;
            %             VoronoiGraph = addnode(VoronoiGraph,size(swarmWorld.cellCenterOfMass,1));
            %             VoronoiGraph.Nodes.centerOfmass = swarmWorld.cellCenterOfMass;
            %
            %
            %             % turn the boundareis of each cell to the form of line
            %             % segments
            %             for kk = 1:size(swarmWorld.cellCenterOfMass,1)
            %                 boundary{kk} = [swarmWorld.voronoiCells{kk}' [swarmWorld.voronoiCells{kk}(end);swarmWorld.voronoiCells{kk}(1:end-1)']];
            %             end
            %
            %             for kk = 1:size(swarmWorld.cellCenterOfMass,1)-1
            %                 for jj = 1:size(boundary{kk},1) % search for shared boudnaries to determine neighborhood
            %                     for ii = kk+1:size(swarmWorld.cellCenterOfMass,1) % search for this boundary in all other cells
            %                         if ismember(boundary{kk}(jj,:),boundary{ii},'rows') || ismember([boundary{kk}(jj,2) boundary{kk}(jj,1)],boundary{ii},'rows') % also need to consider the reversely ordered boundary
            %                             VoronoiGraph = addedge(VoronoiGraph,kk,ii);
            %                         end
            %                     end
            %                 end
            %             end
            %
            %             VoronoiGraph = simplify(VoronoiGraph);
            %             disp('Voronoi Graph')
            %             toc;
            
            % initial cells are given by the cells where agents
            % reside
            % first determine the closest four cell centers and see
            % if the agent is in one of them
            initialNodes = [];
            
            neighborNodes = zeros(swarmModel.N,knnNumber);
            
            for kk = 1:swarmModel.N
                idx = knnsearch(swarmWorld.cellCenterOfMass,swarmState.x(4*(kk-1)+1:4*(kk-1)+2),'K',knnNumber+1); % find knnNumber nearest neighbors
                neighborNodes(kk,:) = idx(2:end);  % check if idx is a row vector
                for ii = 1:5
                    VC = swarmWorld.voronoiVertices(swarmWorld.voronoiCells{idx(ii)},:);
                    initialNodeIndex = find(inpolygon(swarmState.x(4*(kk-1)+1),swarmState.x(4*(kk-1)+2),VC(:,1),VC(:,2)));
                    if initialNodeIndex ~= 0
                        initialNodes = [initialNodes; idx(ii)];
                        break;
                    end
                end
                if length(initialNodes)<kk % if no Voronoi cell contains the agent, just give the cell of closet centroid to the agent.
                    initialNodes = [initialNodes; idx(1)];
                end
            end
            
            
            q = swarmModel.bundleSize; % queue size
            bundleSteps = initialNodes;
            
            % load initial states
            state = zeros(swarmModel.N,4);
            for kk = 1:swarmModel.N
                state(kk,:) = swarmState.x(4*(kk-1)+1:4*(kk-1)+4);
            end
            
            infoSurfaceForAssignment = swarmWorld.mutualInfoSurface;
            
            % build the bundle
            for p = 1:q
                
                if strcmp(swarmModel.taskAllocation,'stepwiseHungarian')
                    % update the target nodes
                    targetNodes = [];
                    for kk = 1:swarmModel.N
                        targetNodes = union(targetNodes,neighborNodes(kk,:));
                    end
                    %                 elseif strcmp(swarmModel.taskAllocation,'stepwiseHungarian_2ndOrder')
                    %                     % update the target nodes
                    %                     targetNodes_1st = [];
                    %                     for kk = 1:swarmModel.N
                    %                         targetNodes_1st = union(targetNodes_1st,neighbors(VoronoiGraph,initialNodes(kk)));
                    %                     end
                    %                     targetNodes = targetNodes_1st;
                    %                     for kk = length(targetNodes_1st)
                    %                         targetNodes = union(targetNodes,neighbors(VoronoiGraph,targetNodes_1st(kk)));
                    %                     end
                end
                % placeholder for likely locations where agents end
                terminalState = cell(length(initialNodes),length(targetNodes));
                
                % placeholder for utility
                utilityMatrix = zeros(length(initialNodes),length(targetNodes));
                % use adjacency matrix for connectivity
                adjacencyMatrix = zeros(swarmModel.N,length(targetNodes));
                
                if strcmp(swarmModel.taskAllocation,'stepwiseHungarian')
                    for kk = 1:length(initialNodes)
                        [~,elements] = ismember(neighborNodes(kk,:),targetNodes);
                        adjacencyMatrix(kk,elements) = 1;
                        for jj = 1:length(targetNodes)
                            if adjacencyMatrix(kk,jj) == 1 % if these two edges are connected
                                [temp1,temp2] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(targetNodes(jj),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state
                                utilityMatrix(kk,jj) = temp1;
                                terminalState{kk,jj} = temp2;
                            else
                                path = [];
                                utilityMatrix(kk,jj) = -10; % if not connected, then the utitily is -10
                            end                            
                        end
                    end
                    %                 elseif strcmp(swarmModel.taskAllocation,'stepwiseHungarian_2ndOrder')
                    %                     adjacencyMatrix_2ndOrder = adjacencyMatrix*adjacencyMatrix;
                    %                     for kk = 1:length(initialNodes)
                    %                         for jj = 1:length(targetNodes)
                    %                             if adjacencyMatrix(initialNodes(kk),targetNodes(jj)) == 1 ||... % if these two edges are connected
                    %                                     adjacencyMatrix_2ndOrder(initialNodes(kk),targetNodes(jj)) == 1 % if the nodes are connected by two edges (2nd order neighbor)
                    %                                 [temp1,temp2] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(targetNodes(jj),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state
                    %                                 utilityMatrix(kk,jj) = temp1;
                    %                                 terminalState{kk,jj} = temp2;
                    %                             else
                    %                                 utilityMatrix(kk,jj) = -10; % if not connected, then the utitily is -10
                    %                             end
                    %                         end
                    %                     end
                end
                
                % Hungarian
                assignmentIndex = munkres(-utilityMatrix);
                initialNodesNew = targetNodes(assignmentIndex);
                % add assignment to bundle
                bundleSteps = [bundleSteps initialNodesNew];
                
                % update the initialNodes
                initialNodes = initialNodesNew;
                
                for kk = 1:swarmModel.N
                    % update the infomation surface sequentially
                    [~,~,infoSurfaceForAssignment] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(initialNodesNew(kk),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state
                    % update initial states for the next step
                    state(kk,:) = terminalState{kk,assignmentIndex(kk)};
                    % update the neighbor nodes
                    idx = knnsearch(swarmWorld.cellCenterOfMass,swarmWorld.cellCenterOfMass(initialNodes(kk),:),'K',knnNumber+1); % find knnNumber nearest neighbors
                    neighborNodes(kk,:) = idx(2:end);  % check if idx is a row vector
                end
            end
            %             disp('bundle built')
            %             toc;
            swarmState.wptList = bundleSteps(:,2:end);
            
            for kk = 1:swarmModel.N
                swarmState.xd(kk) = swarmWorld.cellCenterOfMass(swarmState.wptList(kk,1),1);
                swarmState.yd(kk) = swarmWorld.cellCenterOfMass(swarmState.wptList(kk,1),2);
                swarmState.wptIndex(kk) = 1;
            end
            
            %         % verification ===================
            %         figure;
            %         plot((VoronoiGraph),'XData',swarmWorld.cellCenterOfMass(:,1),'YData',swarmWorld.cellCenterOfMass(:,2))
            %         hold on
            %         for ii = 1:1:100
            %             ind = swarmWorld.voronoiCells{ii};
            %             ind = [ind ind(1)];
            %             plot(swarmWorld.voronoiVertices(ind,1),swarmWorld.voronoiVertices(ind,2),'r');
            %         end
            %
            %         hold on;
            %         plot(swarmWorld.cellCenterOfMass(swarmState.wptList(1,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(1,:),2),'k','LineWidth',2)
            %         plot(swarmWorld.cellCenterOfMass(swarmState.wptList(2,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(2,:),2),'b','LineWidth',2)
            %         plot(swarmWorld.cellCenterOfMass(swarmState.wptList(3,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(3,:),2),'g','LineWidth',2)
            %         plot(swarmWorld.cellCenterOfMass(swarmState.wptList(4,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(4,:),2),'c','LineWidth',2)
            %         % ================================
            %         5;
        end
    else
        error('taskGeneration choice not compatible with stepwiseHungarian or stepwiseHungarian_2ndOrder')
    end
    
elseif strcmp(swarmModel.taskAllocation,'stepwiseHungarian_unique')
    if ( strcmp(swarmModel.taskGeneration,'mutualInfoWpts') )
        % initial cells are given by the cells where agents
        % reside
        % first determine the closest four cell centers and see
        % if the agent is in one of them
        initialNodes = [];
        neighborNodes = zeros(swarmModel.N,swarmModel.knnNumber);
        state = zeros(swarmModel.N,4);        
        for kk = 1:swarmModel.N
            % find knnNumber+1 nearest neighbor (returns indices of
            % cellCenterOfMass)
            idx = knnsearch(swarmWorld.cellCenterOfMass,swarmState.x(4*(kk-1)+1:4*(kk-1)+2),'K',swarmModel.knnNumber+1);
            neighborNodes(kk,:) = idx(2:end);  % check if idx is a row vector
            % neighborNodes(kk,:) = idx(1:end);  % check if idx is a row vector
            for ii = 1:5
                %for ii = 1:swarmModel.knnNumber
                % check if agent kk is in voronoi cell ii
                VC = swarmWorld.voronoiVertices(swarmWorld.voronoiCells{idx(ii)},:);
                initialNodeIndex = find(inpolygon(swarmState.x(4*(kk-1)+1),swarmState.x(4*(kk-1)+2),VC(:,1),VC(:,2)));
                % if it is in the cell set initialNodes
                if initialNodeIndex ~= 0
                    initialNodes = [initialNodes; idx(ii)];
                    break;
                end
            end
            if length(initialNodes) < kk % if no Voronoi cell contains the agent, just give the cell of closet centroid to the agent.
                initialNodes = [initialNodes; idx(1)];
            end
            state(kk,:) = swarmState.x(4*(kk-1)+1:4*(kk-1)+4);
        end                
        q = swarmModel.bundleSize; % queue size
        
        % columns give assignements, rows give each agent,
        bundleSteps = initialNodes;        
        infoSurfaceForAssignment = swarmWorld.mutualInfoSurface;        
        terminateBundle = 0;
        % build the bundle
        for p = 1:q
            if ( terminateBundle == 0 )
                % update the target nodes
                targetNodes = [];
                
                % combine all of the neighbornodes into one set
                for kk = 1:swarmModel.N
                    targetNodes = union(targetNodes,neighborNodes(kk,:));
                end
                
                % remove from this set the nodes that have already been assigned
                totalTasksAssigned = size(bundleSteps,1)*size(bundleSteps,2);
                targetNodes = setdiff(targetNodes, reshape(bundleSteps,totalTasksAssigned,1) );
                
                % placeholder for likely locations where agents end
                terminalState = cell(length(initialNodes),length(targetNodes));
                
                % placeholder for utility
                utilityMatrix = zeros(length(initialNodes),length(targetNodes));
                
                % use adjacency matrix for connectivity
                adjacencyMatrix = zeros(swarmModel.N,length(targetNodes));
                
                % build adjacency and utility matrix
                for kk = 1:length(initialNodes)
                    % check which targetNodes are neighborNodes
                    [~,elements] = ismember(neighborNodes(kk,:),targetNodes);
                    % find nonzero elements
                    nonZeroElems = elements( elements~=0 );
                    % for row kk, set columns of elements to 1 (connected)
                    adjacencyMatrix(kk, nonZeroElems) = 1;
                    % go through all of connected targetNodes
                    for jj = 1:length(targetNodes)
                        if adjacencyMatrix(kk,jj) == 1 % if these two edges are connected
                            [temp1,temp2,~,path] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(targetNodes(jj),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state
                            %[temp1,temp2] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(targetNodes(jj),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state                            
                            utilityMatrix(kk,jj) = temp1;
                            terminalState{kk,jj} = temp2;                            
                        else
                            utilityMatrix(kk,jj) = -1e5; % if not connected, then the utitily is -10
                            path = [];
                        end
                        swarmWorld.pathHistory{kk,p,jj} = path;
                    end
                end
                
                swarmWorld.initialNodes{p} = initialNodes;
                swarmWorld.targetNodes{p} = targetNodes;
                
                % Hungarian
                assignmentIndex = munkres(-utilityMatrix);
                
                % assign new nodes
                initialNodesNew = targetNodes(assignmentIndex);
                
                % add assignment to bundle
                bundleSteps = [bundleSteps initialNodesNew];
                
                % update availableNodes
                
                
                % update the initialNodes
                initialNodes = initialNodesNew;
                
                for kk = 1:swarmModel.N
                    % update the infomation surface sequentially
                    [~,~,infoSurfaceForAssignment] = computeInformationGain(state(kk,:),swarmWorld.cellCenterOfMass(initialNodesNew(kk),:),swarmModel,simParams,infoSurfaceForAssignment,trueWorld); % maybe should also return the ending state
                    % update initial states for the next step
                    
                    % warning: if an error occurs on the following line, it means that
                    % the bundle has already assigned all of the neighbor nodes
                    % for agent kk
                    %
                    % e.g., notice that all neighborNodes of row 2 are contained in bundleSteps
                    % the workaround is to make the bundle size smaller or the
                    % number of neighbors larger
                    % reshape(bundleSteps,1,totalTasksAssigned)
                    %     99    41    15     9    47    58    40    57    66     2    19    65    79     6    81    51    13     7    38    53
                    % neighborNodes =
                    %     66    25    47     1    59     9    22    32    10    41
                    %     58    79    57    65     2    41    66    51     9    47
                    %     38    36    90    31    14    29    93    69    19     7
                    %     65    58    26    57    24    41     6    53     2    94
                    
                    
                    if ( isempty( terminalState{kk,assignmentIndex(kk)} ) )
                        terminateBundle = 1;
                        fprintf('Warning: Bundle building terminating early with %d of %d tasks assigned\n',size(bundleSteps,2),q);
                        break;
                    else
                        state(kk,:) = terminalState{kk,assignmentIndex(kk)};
                    end
                    
                    % update the neighbor nodes
                    idx = knnsearch(swarmWorld.cellCenterOfMass,swarmWorld.cellCenterOfMass(initialNodes(kk),:),'K',swarmModel.knnNumber+1); % find knnNumber nearest neighbors
                    neighborNodes(kk,:) = idx(2:end);  % check if idx is a row vector
                end
            end
        end
        
        %             disp('bundle built')
        %             toc;
        swarmState.wptList = bundleSteps(:,2:end);
        
        for kk = 1:swarmModel.N
            swarmState.xd(kk) = swarmWorld.cellCenterOfMass(swarmState.wptList(kk,1),1);
            swarmState.yd(kk) = swarmWorld.cellCenterOfMass(swarmState.wptList(kk,1),2);
            swarmState.wptIndex(kk) = 1;
        end
        
    end
elseif strcmp(swarmModel.taskAllocation,'stepwiseHungarian_max')
    if ( strcmp(swarmModel.taskGeneration,'mutualInfoWpts') || strcmp(swarmModel.taskGeneration,'likelihoodWpts') )
        % do stepwise Hungarian to generate bundle
        
        % 1. Generate the connectivity graph over the voronoi
        % cells (total time for 100 cells: 0.6s)
        VoronoiGraph = graph;
        VoronoiGraph = addnode(VoronoiGraph,size(swarmWorld.cellCenterOfMass,1));
        VoronoiGraph.Nodes.centerOfmass = swarmWorld.cellCenterOfMass;
        
        VoronoiGraph.Nodes.maxInfo = zeros(size(swarmWorld.cellCenterOfMass,1),1);% initialize a field of max info in each cell
        
        for kk = 1:size(swarmWorld.cellCenterOfMass,1)
            % turn the boundareis of each cell to the form of line segments
            boundary{kk} = [swarmWorld.voronoiCells{kk}' [swarmWorld.voronoiCells{kk}(end);swarmWorld.voronoiCells{kk}(1:end-1)']];
            % count the maximum information in the cell
            cellIndex = inpolygon(trueWorld.xx,trueWorld.yy,...
                swarmWorld.voronoiVertices(swarmWorld.voronoiCells{kk},1),...
                swarmWorld.voronoiVertices(swarmWorld.voronoiCells{kk},2));
            VoronoiGraph.Nodes.maxInfo(kk) = max(max(cellIndex.*swarmWorld.mutualInfoSurface));
            
        end
        
        for kk = 1:size(swarmWorld.cellCenterOfMass,1)-1
            for jj = 1:size(boundary{kk},1) % search for shared boudnaries to determine neighborhood
                for ii = kk+1:size(swarmWorld.cellCenterOfMass,1) % search for this boundary in all other cells
                    if ismember(boundary{kk}(jj,:),boundary{ii},'rows') || ismember([boundary{kk}(jj,2) boundary{kk}(jj,1)],boundary{ii},'rows') % also need to consider the reversely ordered boundary
                        VoronoiGraph = addedge(VoronoiGraph,kk,ii);
                    end
                end
            end
        end
        
        VoronoiGraph = simplify(VoronoiGraph);
        
        % initial cells are given by the cells where agents
        % reside
        % first determine the closest four cell centers and see
        % if the agent is in one of them
        initialNodes = [];
        
        for kk = 1:swarmModel.N
            idx = knnsearch(swarmWorld.cellCenterOfMass,swarmState.x(4*(kk-1)+1:4*(kk-1)+2),'K',5); % find four nearest neighbors
            for ii = 1:5
                VC = swarmWorld.voronoiVertices(swarmWorld.voronoiCells{idx(ii)},:);
                initialNodeIndex = find(inpolygon(swarmState.x(4*(kk-1)+1),swarmState.x(4*(kk-1)+2),VC(:,1),VC(:,2)));
                if initialNodeIndex ~= 0
                    initialNodes = [initialNodes; idx(ii)];
                    break;
                end
            end
            if length(initialNodes)<kk % if no Voronoi cell contains the agent, just give the cell of closet centroid to the agent.
                initialNodes = [initialNodes; idx(1)];
            end
        end
        
        q = swarmModel.bundleSize; % queue size
        bundleSteps = initialNodes;
        
        visitedEdges = [-1 -1];
        
        % build the bundle
        for p = 1:q
            % update the target nodes
            targetNodes = [];
            for kk = 1:swarmModel.N
                targetNodes = union(targetNodes,neighbors(VoronoiGraph,initialNodes(kk)));
            end
            
            % placeholder for likely locations where agents end
            terminalState = cell(length(initialNodes),length(targetNodes));
            
            % placeholder for utility
            utilityMatrix = zeros(length(initialNodes),length(targetNodes));
            % use adjacency matrix for connectivity
            adjacencyMatrix = adjacency(VoronoiGraph);
            for kk = 1:length(initialNodes)
                for jj = 1:length(targetNodes)
                    if adjacencyMatrix(initialNodes(kk),targetNodes(jj)) == 1 % if these two edges are connected
                        % if utility is determined by the maximum
                        % information in the target cell
                        utilityMatrix(kk,jj) = VoronoiGraph.Nodes.maxInfo(targetNodes(jj));
                    else
                        utilityMatrix(kk,jj) = -10; % if not connected, then the utitily is -10
                    end
                end
            end
            
            % Hungarian
            assignmentIndex = munkres(-utilityMatrix);
            initialNodesNew = targetNodes(assignmentIndex);
            % add assignment to bundle
            bundleSteps = [bundleSteps initialNodesNew];
            
            % zero the max info in the assigned cells
            for kk = 1:swarmModel.N
                VoronoiGraph.Nodes.maxInfo(initialNodesNew(kk)) = 0;
            end
            
            % remove the visited edges
            % VoronoiGraph = rmedge(VoronoiGraph,initialNodes,initialNodesNew);
            
            % update visited edges
            visitedEdges = union(visitedEdges,[initialNodes,initialNodesNew],'rows');
            
            % update the initialNodes
            initialNodes = initialNodesNew;
        end
        
        swarmState.wptList = bundleSteps(:,2:end);
        
        for kk = 1:swarmModel.N
            swarmState.xd(kk) = swarmWorld.cellCenterOfMass(swarmState.wptList(kk,1),1);
            swarmState.yd(kk) = swarmWorld.cellCenterOfMass(swarmState.wptList(kk,1),2);
            swarmState.wptIndex(kk) = 1;
        end
        
    else
        error('taskGeneration choice not compatible with stepwiseHungarian_max')
    end
    
elseif strcmp(swarmModel.taskAllocation,'none')
    % for random wpts they are managed only at allocation time
    if strcmp(swarmModel.taskGeneration,'randomWpts')
        % if this is the first time:
        if ( ~isfield(swarmState,'xd') )
            for i = 1:1:swarmModel.N
                swarmState.xd(i) = rand(1,1)*(max(trueWorld.xpoly)-min(trueWorld.xpoly))+min(trueWorld.xpoly);
                swarmState.yd(i) = rand(1,1)*(max(trueWorld.ypoly)-min(trueWorld.ypoly))+min(trueWorld.ypoly);
            end
        end
        % first check if each agent has reached previously assigned
        % waypoint, then only update
        for i = 1:1:swarmModel.N
            xi = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i)];
            xd = [ swarmState.xd(i) swarmState.yd(i) ];
            if ( norm([xi(1) xi(2)] - xd) <= swarmModel.Rsense )
                swarmState.xd(i) = rand(1,1)*(max(trueWorld.xpoly)-min(trueWorld.xpoly))+min(trueWorld.xpoly);
                swarmState.yd(i) = rand(1,1)*(max(trueWorld.ypoly)-min(trueWorld.ypoly))+min(trueWorld.ypoly);
            end
        end
    elseif strcmp(swarmModel.taskGeneration,'randomBoundaryWpts')
        % if this is the first time:
        if ( ~isfield(swarmState,'xd') )
            for i = 1:1:swarmModel.N
                swarmState.xd(i) = rand(1,1)*(max(trueWorld.xpoly)-min(trueWorld.xpoly))+min(trueWorld.xpoly);
                swarmState.yd(i) = rand(1,1)*(max(trueWorld.ypoly)-min(trueWorld.ypoly))+min(trueWorld.ypoly);
            end
        end
        % first check if each agent has reached previously assigned
        % waypoint, then only update
        for i = 1:1:swarmModel.N
            xi = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i)];
            xd = [ swarmState.xd(i) swarmState.yd(i) ];
            if ( norm([xi(1) xi(2)] - xd) <= swarmModel.Rsense )
                swarmState.xd(i) = rand(1,1)*(max(trueWorld.xpoly)-min(trueWorld.xpoly))+min(trueWorld.xpoly);
                swarmState.yd(i) = rand(1,1)*(max(trueWorld.ypoly)-min(trueWorld.ypoly))+min(trueWorld.ypoly);
            end
        end
        %     elseif strcmp(swarmModel.taskGeneration,'lawnmower')
        %         % first check if each agent has reached previously assigned
        %         % waypoint, then only update
        %         for i = 1:1:swarmModel.N
        %             xi = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i)];
        %             xd = [ swarmState.xd(i) swarmState.yd(i) ];
        %             if ( norm([xi(1) xi(2)] - xd) <= swarmModel.Rsense/10 )
        %                 disp('Updating Wpt');
        %                 swarmState.wptIndex(i) = swarmState.wptIndex(i) + 1;
        %                 if ( swarmState.wptIndex(i) > swarmState.numWpts )
        %                     swarmState.wptIndex(i) = 1;
        %                     fprintf('Quad %i completed lawnmower, restarting \n',i);
        %                 end
        %                 swarmState.xd(i,1) = swarmState.wptListX(swarmState.wptIndex(i),i);
        %                 swarmState.yd(i,1) = swarmState.wptListY(swarmState.wptIndex(i),i);
        %             end
        %         end
    end
elseif strcmp(swarmModel.taskAllocation,'Hungarian')
    % Hungarian algorithm goes here
    
    % Sheng: just combine the energy and penalty method here
    if strcmp(swarmModel.taskGeneration,'frontierWpts')
        switch swarmModel.communicationTopology
            case 'allToAll'
                % first assign random waypoints to all agents
                swarmState.xd = rand(1,1)*(max(trueWorld.xpoly)-min(trueWorld.xpoly))+min(trueWorld.xpoly);
                swarmState.yd = rand(1,1)*(max(trueWorld.ypoly)-min(trueWorld.ypoly))+min(trueWorld.ypoly);
                
            case 'centralized'
                % first assign random waypoints to all agents
                swarmState.xd = rand(swarmModel.N,1)*(max(trueWorld.xpoly)-min(trueWorld.xpoly))+min(trueWorld.xpoly);
                swarmState.yd = rand(swarmModel.N,1)*(max(trueWorld.ypoly)-min(trueWorld.ypoly))+min(trueWorld.ypoly);
        end
        % Then, if task has been generated, then run a task allocation algorithm
        % to assign tasks to agents
        % This approach makes sure all agents are assigned waypoint even if
        % the number of tasks is less than agents
        
        swarmState.xd = (trueWorld.maxX+trueWorld.minX)/2*ones(size(swarmState.xd));
        swarmState.yd = (trueWorld.maxY+trueWorld.minY)/2*ones(size(swarmState.yd));
        
        if ~isempty(tasks)
            % if tasks are not empty, then generate the taskTable
            taskTable = generateTaskTable(swarmState.x,tasks,swarmModel,simParams,swarmWorld);
            if ( strcmp(swarmModel.mapping.method,'frontierAndBlob') )
                % scale the cost for subblob centroid
                taskTable(:,length(swarmWorld.frontierIndex)+1:end) = taskTable(:,length(swarmWorld.frontierIndex)+1:end)*swarmModel.mapping.blobCostScale;
            end
            % at this point making a request to auction
            % the return most likely would be the assignment queue or a
            % currently active task.
            %
            
            % use Munkres method for task allocation
            taskAllocation = munkres(taskTable);
            [assignedAgent,~] = find(~~taskAllocation');
            swarmState.xd(assignedAgent) = tasks(taskAllocation(assignedAgent)',1);
            swarmState.yd(assignedAgent) = tasks(taskAllocation(assignedAgent)',2);
        else
            % if there are no tasks, then just assign all agents to the
            % center (or switch to other actions)
            swarmState.xd = (trueWorld.maxX+trueWorld.minX)/2*ones(size(swarmState.xd));
            swarmState.yd = (trueWorld.maxY+trueWorld.minY)/2*ones(size(swarmState.yd));
        end
        
    elseif strcmp(swarmModel.taskGeneration,'mutualInfoWpts')
        % build a distance matrix
        %     	M = length(tasks);
        %     	D = zeros(swarmModel.N,M);
        %     	for i = 1:1:swarmModel.N % rows are agents
        %         	for j = 1:1:M % columns are task
        %             	D(i,j) = norm(tasks(j,:)-[swarmState.x(4*i-3) swarmState.x(4*i-2)]);
        %         	end
        %     	end
        %     	% build a reward matrix based on cell mass
        %     	R = zeros(swarmModel.N,M);
        %     	for i = 1:1:swarmModel.N
        %        	R(i,:) = swarmWorld.cellMass;
        %     	end
        %     	% Cost
        %     	R = scaleMatrix(R); % scale matrix so smallest entry is 0, largest is 1
        %     	D = scaleMatrix(D); % scale matrix so smallest entry is 0, largest is 1
        %     	C = -R; % ignore distance
        % solve a single-task assignment problem using the munkres/hungarian algorithm
        % assignment = munkres(C);
        
        tStart = tic;
        disp('Generating task table...')
        taskTable = generateTaskTable(swarmState.x,tasks,swarmModel,simParams,swarmWorld,trueWorld);
        fprintf('generateTaskTable took %3.3f s\n',toc(tStart));
        swarmWorld.taskTable = taskTable;
        assignment = munkres(taskTable);
        % set desired waypoints
        swarmState.xd = tasks(assignment,1);
        swarmState.yd = tasks(assignment,2);
        
    end
    
    
elseif strcmp(swarmModel.taskAllocation,'Auctioneer')
    % wrapper for Ken's auctioneer algorithm goes here
    
    % MEMO:
    % 1. A task manager is needed and required to
    %   - assign unique id's to tasks
    %   - keep track of the tasks in an agent's bid bundle that is/will be serviced
    % 2. Inside the wrapper, a copy of agents and tasks is maintained with
    %    the variable definition for the auctioneer algorithm by Ken.
    
    numAvailableAgents = swarmModel.N;
    
    % get available tasks
    numTasks = size(tasks,1); %number of initial tasks to seed the environment with
    
    availableTasks = ADCA_AvailableTaskQueue();
    
    for m = 1:numTasks
        taskContainer = ADCA_TaskContainer();
        
        taskDescriptor = ADCA_TaskLoiterDescriptor(0, m);
        taskDescriptor.LoiterLocation = swarmWorld.OperationalWorld.loadTaskLocation([tasks(m,:) 0]);% Sheng: load locations of tasks
        
        taskContainer.setTaskDescriptor(taskDescriptor);
        %taskContainer.m_BeginningState.POSITION = OperationalWorld.getRandomLocation();
        %taskContainer.m_EndingState.POSITION = OperationalWorld.getRandomLocation();
        
        availableTasks.appendTaskToList(taskContainer);
        
        %     taskDescriptor.graphTaskProperties(EnvironmentAxes, OperationalWorld.colorMapTasks);
    end
    %Notify all of the agents of the currently available tasks
    for n = 1:numAvailableAgents
        swarmWorld.roboticAgents(n,1).newlyAvailableTasks(availableTasks);
    end
    
    % get available agents: right now assume all the agents are available
    
    % bid and consensus
    consensusAchieved = false;
    abort = false;
    
    auctionIteration = 1;
    lastAuctionChange = auctionIteration - 1;
    
    while ((consensusAchieved == false) && (abort == false))
        fprintf('\nWe are about to communicate and perform a consensus step\n');
        %Communicate to all the agents about the current belief
        ADCA_MeshCommunication(swarmWorld.roboticAgents,swarmWorld.communicationMatrix);
        
        for n = 1:numAvailableAgents
            fprintf('The perceived leaders of agent %i. \n',swarmWorld.roboticAgents(n,1).m_RoboticParameters.AGENT_ID);
            swarmWorld.roboticAgents(n,1).m_MasterTaskList.printTaskLeaders();
            instantaneousAgentUtility = swarmWorld.roboticAgents(n,1).m_MasterTaskList.computeAgentUtility(n);
            if(isKey(swarmWorld.agentUtilityMap,num2str(n)))
                swarmWorld.agentUtilityMap(num2str(n)) = [swarmWorld.agentUtilityMap(num2str(n)),instantaneousAgentUtility];
            else
                swarmWorld.agentUtilityMap(num2str(n)) = instantaneousAgentUtility;
            end
        end
        
        fprintf('We are about to build the appropriate bundles.\n');
        %Allow the agents to establish each of their own desired bundles
        for n = 1:numAvailableAgents
            [generatedBundle, newlyFormedBundle] = ADCA_BuildBundle(swarmWorld.roboticAgents(n,1),datetime('now'),swarmWorld.roboticAgents(n,1).getConcludingState(),swarmWorld.roboticAgents(n,1).getAvailableTasks());
            if(newlyFormedBundle)
                lastAuctionChange = auctionIteration;
            end
        end
        
        for n = 1:numAvailableAgents
            fprintf('The perceived task order of agent %i. \n',swarmWorld.roboticAgents(n,1).m_RoboticParameters.AGENT_ID);
            swarmWorld.roboticAgents(n,1).m_MasterTaskList.printCurrentAssignments();
        end
        
        if((auctionIteration - lastAuctionChange) > 0)
            consensusAchieved = true;
        elseif((auctionIteration - lastAuctionChange) > (2*numTasks))
            consensusAchieved = true;
        end
        
        fprintf('Pause after auction iteration %i. \n',auctionIteration);
        auctionIteration = auctionIteration + 1;
        
    end
end
end

