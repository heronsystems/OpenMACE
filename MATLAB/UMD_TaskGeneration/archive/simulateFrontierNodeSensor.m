function frontierIndex = simulateFrontierNodeSensor(exploredGraph, trueWorldGraph, frontierIndex, numberPreviouslyExploredNodes)
% Sheng: first check previous frontier nodes.
% If each frontier node's number of neighbors < total number of
% neighbors in truemap, then it's still a frontier;
% Also need to combine newly added nodes into the same process of
% frontier marking.
if isempty(frontierIndex) && ~isempty(exploredGraph.Nodes)
    % first time edit frontierIndex
    for k = 1:length(exploredGraph.Nodes.trueGraphIndex)
        if length(outedges(exploredGraph,k)) < length(outedges(simplify(trueWorldGraph),exploredGraph.Nodes.trueGraphIndex(k)))
            frontierIndex = [frontierIndex;exploredGraph.Nodes.trueGraphIndex(k)];
        elseif length(outedges(exploredGraph,k)) > length(outedges(trueWorldGraph,exploredGraph.Nodes.trueGraphIndex(k)))
            % just for debug purpose
            error('something is wrong in the adjacency matrix (during frontierIndex initialization)');
        end
    end
elseif isempty(exploredGraph.Nodes)
    % do nothing
else  
    % update frontierIndex by first check frontierIndex of last
    % iteration
    expiredFrontierIndex = []; % This variable is used to store the index of expired nodes
    for k = 1:length(frontierIndex)
        nodeIndexInExploredGraph = find(exploredGraph.Nodes.trueGraphIndex == frontierIndex(k));
        if length(outedges(exploredGraph,nodeIndexInExploredGraph)) == length(outedges(trueWorldGraph,frontierIndex(k)))
            % this is not a frontier node, store its index in expiredFrontierIndex
            expiredFrontierIndex = [expiredFrontierIndex;k];
        elseif length(outedges(exploredGraph,nodeIndexInExploredGraph)) > length(outedges(trueWorldGraph,frontierIndex(k)))
            % just for debug purpose
            error('something is wrong in the adjacency matrix (during updating previous frontierIndex)');
        end
    end
    % eliminate expired frontiers
    frontierIndex(expiredFrontierIndex) = [];
    
    % just try to catch an exception
    if numberPreviouslyExploredNodes > length(exploredGraph.Nodes.trueGraphIndex)
        error("Exception!");
    end
    
    % then examine newly added nodes
    for k = numberPreviouslyExploredNodes+1:length(exploredGraph.Nodes.trueGraphIndex)
        if length(outedges(exploredGraph,k)) < length(outedges(trueWorldGraph,exploredGraph.Nodes.trueGraphIndex(k)))
            frontierIndex = [frontierIndex;exploredGraph.Nodes.trueGraphIndex(k)];
        elseif length(outedges(exploredGraph,k)) > length(outedges(trueWorldGraph,exploredGraph.Nodes.trueGraphIndex(k)))
            % just for debug purpose
            error('something is wrong in the adjacency matrix (during registering newly added frontierIndex)');
        end
    end
    
end
end
