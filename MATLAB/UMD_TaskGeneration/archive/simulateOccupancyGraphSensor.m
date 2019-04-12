function [discoveredNodes, discoveredEdges, cellStateMat] = simulateOccupancyGraphSensor(exploredGraph, trueWorldGraph, discoveredCells, bin2NodeID, cellStateMat, numNodesMat)

discoveredNodes = [];
for i = 1:1:size(discoveredCells,1)
    bx = discoveredCells(i,1);
    by = discoveredCells(i,2);
    
    if ( cellStateMat(by,bx) == 2 )
        % update the cell state mat:
        % 1 - if it contains a node of the occupancy graph
        % 0 - if it is an obstacle cell
        cellStateMat(by,bx) = (numNodesMat(by,bx)~= 0);  %
        if ( cellStateMat(by,bx) == 1 )
            % determine index of this node in the trueWorld
            discoveredNodeIndex = bin2NodeID(by,bx);    
            discoveredNodes = [discoveredNodes; discoveredNodeIndex];
            
            % discover edges
            % add this new node to the exploredGraph
            NodeProps = table(discoveredNodeIndex,'VariableNames', {'trueGraphIndex'});
            exploredGraph = addnode(exploredGraph,NodeProps);
            
            % determine the neighbor nodes of the newlyDiscoveredNode
            [neighborIndex] = neighbors(trueWorldGraph,discoveredNodeIndex);
            
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
    end
end

discoveredNodes = unique(discoveredNodes);


end

