function [swarmWorld, numberPreviouslyExploredNodes] = buildOccupancyGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, nodeCells )

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

% save number of previously explored nodes
if ( ~isempty(swarmWorld.exploredGraph.Nodes) )
    switch swarmModel.mappingSensorType
        case 'perfect'
            numberPreviouslyExploredNodes = length(swarmWorld.exploredGraph.Nodes.trueGraphIndex);
        case 'noisy'
            numberPreviouslyExploredNodes = length(swarmWorld.exploredGraph.Nodes.bx);
    end
else
    numberPreviouslyExploredNodes = 0;
end

% determine number of target state space graph nodes (before)
if ( ~isempty(swarmWorld.targetStateSpaceGraph) )
    numNodesGtssPrior = numnodes(swarmWorld.targetStateSpaceGraph);
else
    numNodesGtssPrior = 0;
end

newlyAdded  = [];

if (~isempty(nodeCells))
    for i = 1:1:size(nodeCells,1)
        % nodeCells is the subset of newly discovered cells that contain
        % nodes
        bx = nodeCells(i,1);
        by = nodeCells(i,2);
        
        switch swarmModel.mappingSensorType
            case 'perfect'
                discoveredNodeIndex = trueWorld.bin2NodeID(by,bx);
                
                % discover edges
                % add this new node to the exploredGraph
                NodeProps = table(discoveredNodeIndex,'VariableNames', {'trueGraphIndex'});
                swarmWorld.exploredGraph = addnode(swarmWorld.exploredGraph,NodeProps);
                
                % determine the neighbor nodes of the newlyDiscoveredNode
                [neighborIndex] = neighbors(trueWorld.G_env,discoveredNodeIndex);
                
                % Index of the new node is equal to the number of nodes since it is the
                % last one added
                idNew = numnodes(swarmWorld.exploredGraph);
                
                % determine if new edges are discovered, by
                % checking whether neighbor nodes is connected to
                % the newly discovered node
                for k = 1:length(neighborIndex)
                    % record the integer number of newly discovered node and neighbor node
                    idNeighbor = find(swarmWorld.exploredGraph.Nodes.trueGraphIndex == neighborIndex(k));
                    
                    % if neighborIndex(k) \in exploredGraph.Nodes
                    % && there's no edge between the newly discovered node and neighbor(k
                    if sum(swarmWorld.exploredGraph.Nodes.trueGraphIndex == neighborIndex(k)) && ...
                            ~sum(findedge(swarmWorld.exploredGraph,[idNew],[idNeighbor]))
                        % then add this new edge
                        swarmWorld.exploredGraph = addedge(swarmWorld.exploredGraph,[idNew],[idNeighbor]);
                    end
                end
                % augment target state space
                % Sheng uncomment the following when finished decentralization
                [swarmWorld] = buildTargetStateSpaceGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, idNew );
            case 'noisy'
                
                % discover edges
                % add this new node to the exploredGraph
                NodeProps = table(bx, by ,'VariableNames', {'bx', 'by'});
                swarmWorld.exploredGraph = addnode(swarmWorld.exploredGraph,NodeProps);
                idNew = numnodes(swarmWorld.exploredGraph);
                swarmWorld.bin2NodeIDexplored(by,bx) = idNew;
                % check if any of the  4 ells around it are detected
                [bxNeighbor, byNeighbor] = getNeighborCells( bx, by, trueWorld.numBinsX, trueWorld.numBinsY );
                for k = 1:1:length(bxNeighbor)
                    if ( swarmWorld.cellDetMat( byNeighbor(k) , bxNeighbor(k) ) == 1 )
                        % but to add edge, the detected cell must already
                        % be in the graph
                        idNeighbor = swarmWorld.bin2NodeIDexplored( byNeighbor(k) , bxNeighbor(k) );
                        
                        if ( idNeighbor ~= 0 ) % node has not yet been added to graph, skip
                            if ( findedge(swarmWorld.exploredGraph,idNew,idNeighbor)==0 ) % if edge already exists, skip
                                swarmWorld.exploredGraph = addedge(swarmWorld.exploredGraph,idNew,idNeighbor);
                            end
                        end
                    end
                end
                [swarmWorld] = buildTargetStateSpaceGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, idNew );
                
        end
        
        % Sheng uncommented the following line when he tried to run the
        % auctioneer with decentralized situational awareness.
%          [targetStateSpaceGraph, Mpc2s, Mc, Mp, Q, log_likelihood] = buildTargetStateSpaceGraphOnTheFly(targetStateSpaceGraph, exploredGraph, trueWorldGraph, idNew, Mpc2s, Mc, Mp, Q, log_likelihood, probStopping, m , d  );
    end
    
end



switch swarmModel.mappingSensorType
    case 'perfect'
        % determine number of target state space graph nodes (after)
        numNodesGtssPost = numnodes(swarmWorld.targetStateSpaceGraph);
        log_pNom = log ( (1-probAbsentPrior)/numNodesGtssPost / probAbsentPrior );
        
        % add baseline probability to the new nodes
        for n = numNodesGtssPrior+1:1:numNodesGtssPost
            swarmWorld.log_likelihood(n) = log_pNom;
        end
    case 'noisy'
%         % add likelihood based on prior target sensor
%         numNodesGtssPost = numnodes(swarmWorld.targetStateSpaceGraph);
%         for n = numNodesGtssPrior+1:1:numNodesGtssPost
%             curNode = swarmWorld.targetStateSpaceGraph.Nodes.curNode(n);
%             bx = swarmWorld.exploredGraph.Nodes.bx(curNode);
%             by = swarmWorld.exploredGraph.Nodes.by(curNode);
%             LR = swarmWorld.O(by,bx) / swarmWorld.U(by,bx);
%             %fprintf('Adding likelihood from node (%d,%d) = %6.6f , logLR = %3.3f , U = %3.1f , O = %3.1f \n',by,bx,LR, log(LR), swarmWorld.U(by,bx), swarmWorld.O(by,bx) );
%             swarmWorld.log_likelihood(n) = log ( LR );
%         end
        
        % determine number of target state space graph nodes (after)
        numNodesGtssPost = numnodes(swarmWorld.targetStateSpaceGraph);
        
        
        % Approach 1 : adaptive, initialize based on size of current state space 
        %log_pNom = log ( (1-swarmModel.probAbsentPrior)/numNodesGtssPost / swarmModel.probAbsentPrior );
        
        % Approach 2 : static, initialize base on estimate of state-space size
        numNodesEst = (swarmModel.numNodesEstPercent*(trueWorld.numBinsX*trueWorld.numBinsY).^2) + (trueWorld.numBinsX*trueWorld.numBinsY); %
        log_pNom = log ( (1-swarmModel.probAbsentPrior)/numNodesEst / swarmModel.probAbsentPrior );
        
        
        
        % add baseline probability to the new nodes
        for n = numNodesGtssPrior+1:1:numNodesGtssPost
            swarmWorld.log_likelihood(n) = log_pNom;
        end
        %if ( numNodesGtssPrior ~= numNodesGtssPost )
           %fprintf('Added logLR= %6.6f, LR= %6.6f, numNodesPrior= %d,  numNodesPost = %d, probAbsentPrior = %3.3f \n', log_pNom, exp(log_pNom), numNodesGtssPrior, numNodesGtssPost, swarmModel.probAbsentPrior); 
        %end
        
%         figure(1)
%         plot(swarmWorld.log_likelihood)
%         title('Log likelihood')
%         disp('sum LR')
%         sum(exp(swarmWorld.log_likelihood))
end