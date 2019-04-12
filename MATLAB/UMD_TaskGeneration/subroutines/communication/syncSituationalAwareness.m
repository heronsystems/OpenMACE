function swarmWorld = syncSituationalAwareness(swarmWorld,swarmState,swarmModel,trueWorldGraph)
% This function syncronizes the situational awareness of each agent
% according the the communication topology determined by
% swarmWorld.communicationMat.

% update communication matrix

% define message 
%   swarmWorld
for i = 1:swarmModel.N
    % update each agent's own state(make it a function)
    swarmWorld{i}.allAgentState(:,i) = swarmState{i}.x'; % update its own state
end
for i = 1:swarmModel.N-1
    
    for j = i+1:swarmModel.N
        if swarmWorld{1}.communicationMat(i,j) == 1
            % assume all the agents have the identicay communicationMat
            % [swarmWorld{i}, swarmWorld{j}] = exchangeMessage(swarmWorld{i}, swarmWorld{j});
            
            
            
            % update the state of other agents
            swarmWorld{i}.allAgentState(:,j) = swarmState{j}.x';
            swarmWorld{j}.allAgentState(:,i) = swarmState{i}.x';
            
            % exchange cellStateMat
            cellStateMati = swarmWorld{i}.cellStateMat;
            cellStateMatj = swarmWorld{j}.cellStateMat;
            
            swarmWorld{i}.cellStateMat((find(cellStateMatj==1))) = 1;
            swarmWorld{i}.cellStateMat((find(cellStateMatj==0))) = 0;
            swarmWorld{j}.cellStateMat((find(cellStateMati==1))) = 1;
            swarmWorld{j}.cellStateMat((find(cellStateMati==0))) = 0;
            
            % merge exploredGraph
            exploredGraphi = swarmWorld{i}.exploredGraph;
            exploredGraphj = swarmWorld{j}.exploredGraph;
            
            % initialize new graph
            commonGraph = graph;
            
            % union all the explored nodes
            commonNodes = union(exploredGraphi.Nodes.trueGraphIndex,exploredGraphj.Nodes.trueGraphIndex);
            
            % update the nodes of the commonGraph
            NodeProps = table(commonNodes,'VariableNames', {'trueGraphIndex'});
            commonGraph = addnode(commonGraph,NodeProps);
            
            % traverse all the nodes to add edges to the commonGraph
            for k = 1:length(commonNodes)
                [nodeIndexi,~] = find(exploredGraphi.Nodes.trueGraphIndex == commonNodes(k)); % nodeIndexi is nodeid
                [nodeIndexj,~] = find(exploredGraphj.Nodes.trueGraphIndex == commonNodes(k));
                if ~isempty(nodeIndexi)
                    % add all the edges from exploredGraphi to the common
                    % graph
                    [~,neighborNodes] = outedges(exploredGraphi,nodeIndexi); % neighborNodes are nodeid
                    neighborNodeIndex = exploredGraphi.Nodes.trueGraphIndex(neighborNodes); % neighborNodeIndex is truegraphindex
                    [~,neighborNodeIndex] = ismember(neighborNodeIndex,commonGraph.Nodes.trueGraphIndex); % neighborNodeIndex is nodeid
                    commonGraph = addedge(commonGraph,k*ones(size(neighborNodeIndex)),neighborNodeIndex);                    
                end
                if ~isempty(nodeIndexj)
                    % add all the edges from exploredGraphi to the common
                    % graph
                    [~,neighborNodes] = outedges(exploredGraphj,nodeIndexj); % neighborNodes are nodeid
                    neighborNodeIndex = exploredGraphj.Nodes.trueGraphIndex(neighborNodes); % neighborNodeIndex is truegraphindex
                    [~,neighborNodeIndex] = ismember(neighborNodeIndex,commonGraph.Nodes.trueGraphIndex); % neighborNodeIndex is nodeid
                    commonGraph = addedge(commonGraph,k*ones(size(neighborNodeIndex)),neighborNodeIndex);          
                end
            end
            commonGraph = simplify(commonGraph);
            % note: building commonGraph is not finished
                        
            % sync frontiers
            frontierIndex = union(swarmWorld{i}.frontierIndex,swarmWorld{j}.frontierIndex);
            
            expiredFrontierIndex = []; % This variable is used to store the index of expired nodes
            for k = 1:length(frontierIndex)
                nodeIndexInExploredGraph = find(commonGraph.Nodes.trueGraphIndex == frontierIndex(k));
                if length(outedges(commonGraph,nodeIndexInExploredGraph)) == length(outedges(trueWorldGraph,frontierIndex(k)))
                    % this is not a frontier node, store its index in expiredFrontierIndex
                    expiredFrontierIndex = [expiredFrontierIndex;k];
                elseif length(outedges(commonGraph,nodeIndexInExploredGraph)) > length(outedges(trueWorldGraph,frontierIndex(k)))
                    % just for debug purpose
                    error('something is wrong in the adjacency matrix (during updating previous frontierIndex)');
                end
            end
            frontierIndex(expiredFrontierIndex) = [];
            
            expiredFrontierIndex = []; % This variable is used to store the index of expired nodes
            % The following nested for-loops remove the adjacent frontiers
            % that are adjacent in trueWorldGraph but they come from
            % different exploredGraphs
            for k = 1:length(frontierIndex)
                for m = k+1:length(frontierIndex)
                    % if the edge between k-th and m-th frontier exists in
                    % the true graph (i.e., they are adjacent) && they come
                    % from different exploredGraphs, then we consider both as non-frontiers.
                    if (findedge(trueWorldGraph,frontierIndex(k),frontierIndex(m)) && ...
                         ~(prod(ismember([frontierIndex(k),frontierIndex(m)],exploredGraphi.Nodes.trueGraphIndex)) || ...
                               prod(ismember([frontierIndex(k),frontierIndex(m)],exploredGraphj.Nodes.trueGraphIndex))))
                        expiredFrontierIndex = [expiredFrontierIndex;k;m];
                        % but need to supply this edge to the merged graph
                        [nodeId1,~] = find(commonGraph.Nodes.trueGraphIndex == frontierIndex(k));
                        [nodeId2,~] = find(commonGraph.Nodes.trueGraphIndex == frontierIndex(m));
                        commonGraph = addedge(commonGraph,nodeId1,nodeId2); 
                    end
                end
            end
            frontierIndex(expiredFrontierIndex) = [];
            
            swarmWorld{i}.frontierIndex = frontierIndex;
            swarmWorld{j}.frontierIndex = frontierIndex;
            
            % building commonGraph is finished
            swarmWorld{i}.exploredGraph = commonGraph;
            swarmWorld{j}.exploredGraph = commonGraph;
            
            % sync availability of tasks
            commonAvailableTask = swarmWorld{i}.availableTask.*swarmWorld{j}.availableTask;
            swarmWorld{i}.availableTask = commonAvailableTask;
            swarmWorld{j}.availableTask = commonAvailableTask;
        end
        
    end
end
% iterate through send/receive with each agent

% syncCellStateMat
% syncOccupancyGraph: unions of nodes, but may have to add edges

% syncFrontierNodes: issue when detecting frontier from left and right


% syncTargetStateSpaceGraph: 
% syncLikelihood: conflict occurs when two agents have same node with
%   different likelihood and were previously unaware of each other

