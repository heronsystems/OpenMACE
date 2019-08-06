function [swarmWorld] = buildOccupancyGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, nodeCells )

% Input:,
%   targetStateSpaceGraph
%   exploredGraph
%   trueWorldGraph
%   bin2NodeID
%   Mpc2s
%   Mc
%   Mp
%   nodeCells : (bx,by)
%   Q : state transition matrix
%   log_likelihood
%   probStopping : parameter used by target motion model
%   m : parameter used by target motion model
%   d : parameter used by target motion model

% Output:
%   targetStateSpaceGraph
%   exploredGraph
%   Mpc2s
%   Mc
%   Mp
%   Q
%   log_likelihood
%   numberPreviouslyExploredNodes

% determine number of target state space graph nodes (before)
if ( ~isempty(swarmWorld.targetStateSpaceGraph) )
    numNodesGtssPrior = numnodes(swarmWorld.targetStateSpaceGraph);
else
    numNodesGtssPrior = 0;
end

if (~isempty(nodeCells))
    for i = 1:1:size(nodeCells,1)
        % nodeCells: subset of newly discovered cells that contain nodes
        bx = nodeCells(i,1); % bin index 
        by = nodeCells(i,2);
        idNewTrue = trueWorld.bin2NodeID(by, bx);
        % add this new node to the exploredGraph
        if ( idNewTrue == 0 )
            % if this is a falsely delcared note, place it randomly in the
            % cell 
            nodeX = trueWorld.xcp(bx) + (rand()-0.5)*trueWorld.binWidth;
            nodeY = trueWorld.ycp(by) + (rand()-0.5)*trueWorld.binWidth;           
            elsee
            nodeX = trueWorld.nodeX(idNewTrue);
            nodeY = trueWorld.nodeY(idNewTrue);
        end
        NodeProps = table(bx, by, nodeX, nodeY, 'VariableNames', {'bx', 'by', 'nodeX', 'nodeY'});
        swarmWorld.exploredGraph = addnode(swarmWorld.exploredGraph,NodeProps);
        idNew = numnodes(swarmWorld.exploredGraph);
        swarmWorld.bin2NodeIDexplored(by,bx) = idNew;
        % add edges
        % check if any of the  8 cells around it are detected
        [bxNeighbor, byNeighbor] = getNeighborCells( bx, by, trueWorld.numBinsX, trueWorld.numBinsY );
        for k = 1:1:length(bxNeighbor)
            if ( swarmWorld.cellDetMat( byNeighbor(k) , bxNeighbor(k) ) == 1 )
                % to add edge, the detected cell must already in graph
                % check if neighbor is in graph 
                idNeighbor = swarmWorld.bin2NodeIDexplored( byNeighbor(k) , bxNeighbor(k) );                
                if ( idNeighbor ~= 0 ) % node is in graph
                    if ( findedge(swarmWorld.exploredGraph,idNew,idNeighbor)==0 ) % if edge already exists, skip
                        % check that indeed, the true world has this edge                             
                        idNeighborTrue = trueWorld.bin2NodeID(byNeighbor(k), bxNeighbor(k));
                        % if one of the nodes is zero, then this is a
                        % spurious node (not a real one)
                        % we add the edge to any nearby nodes
                        if ( idNewTrue == 0 || idNeighborTrue == 0 )
                            swarmWorld.exploredGraph = addedge(swarmWorld.exploredGraph,idNew,idNeighbor);
                        elseif ( trueWorld.A_env(idNewTrue,idNeighborTrue)==1 ) 
                            swarmWorld.exploredGraph = addedge(swarmWorld.exploredGraph,idNew,idNeighbor);
                        end
                    end
                end % node has not yet been added to graph, skip
            end
        end
        [swarmWorld] = buildTargetStateSpaceGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, idNew );
    end
    
end

% determine number of target state space graph nodes (after)
numNodesGtssPost = numnodes(swarmWorld.targetStateSpaceGraph);

% initialize based on estimate of state-space size
numNodesEst = (swarmModel.numNodesEstPercent*(trueWorld.numBinsX*trueWorld.numBinsY).^2) + (trueWorld.numBinsX*trueWorld.numBinsY); %
log_pNom = log ( (1-swarmModel.probAbsentPrior)/numNodesEst / swarmModel.probAbsentPrior );

% add baseline probability to the new nodes
swarmWorld.log_likelihood(numNodesGtssPrior+1:1:numNodesGtssPost) = log_pNom;

end