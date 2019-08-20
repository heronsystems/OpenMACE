function [exploredGraph,targetStateSpaceGraph,Mcp2s,Mc,Mp] = updateExploredGraph(bx,by,bin2NodeID,exploredGraph,trueWorld,targetStateSpaceGraph,Mcp2s,Mc,Mp)
%
% Description: update the explored graph
%
% Input:
%   bx,by: index of current controlPt
%   bin2NodeID: a matrix that maps the index of controlPt back node index
%   exploredGraph: a graph object that records the explored graph
%   A: adjacency matrix of the complete map (graph)
%   targetStateSpace: a Ns x 2 matrix with each row indicating a possible
%       state for the target. The first row indicates the current node
%       index, the second row is the previous node index.
%
% Output:
%   exploredGraph: a graph object that records the explored graph
%   targetStateSpace: a Ns x 2 matrix with each row indicating a possible
%       state for the target. The first row indicates the current node
%       index, the second row is the previous node index.
%
% Sheng Cheng, 2018

% Sheng: NEW
% Now we do not record pseudo-edges (edges connects one explored and one
% unexplored node).
% And we use attribute 'frontier' to denote if a node is a frontier

% The procedure is the following
% 1. Add new node.
% 2. And add edges to whichever neighbor node that currently belongs to the explored graph.

% store the index of newly discovered node in newlyDiscoveredNodeIndex
newlyDiscoveredNodeIndex = bin2NodeID(by,bx);

% add this new node to the exploredGraph
NodeProps = table(newlyDiscoveredNodeIndex,'VariableNames', {'trueGraphIndex'});
exploredGraph = addnode(exploredGraph,NodeProps);

% determine the neighbor nodes of the newlyDiscoveredNode
[neighborIndex] = neighbors(trueWorld.G_env,newlyDiscoveredNodeIndex);

% Index of the new node is equal to the number of nodes since it is the
% last one added
idNew = numnodes(exploredGraph);

% determine if new edges are discovered, by
% checking whether neighbor nodes is connected to
% the newly discovered node
for k = 1:length(neighborIndex)
    % record the integer number of newly discovered node and neighbor node
    idNeighbor = find(exploredGraph.Nodes.trueGraphIndex == neighborIndex(k));
    % if neighborIndex(k) \in exploredGraph.Nodes
    % && there's no edge between the newly discovered node and neighbor(k
    if sum(exploredGraph.Nodes.trueGraphIndex == neighborIndex(k)) && ...
            ~sum(findedge(exploredGraph,[idNew],[idNeighbor]))
        % then add this new edge
        exploredGraph = addedge(exploredGraph,[idNew],[idNeighbor]);
    end
end

end

