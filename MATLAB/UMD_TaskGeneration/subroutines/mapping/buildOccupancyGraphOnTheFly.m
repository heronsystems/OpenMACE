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

% determine number of target state space graph nodes (before update)
if ( ~isempty(swarmWorld.targetStateSpaceGraph) )
    numNodesGtssPrior = numnodes(swarmWorld.targetStateSpaceGraph);
else
    numNodesGtssPrior = 0;
end

% nodeCells: subset of newly discovered cells that contain nodes
if (~isempty(nodeCells))
    for i = 1:1:size(nodeCells,1)
        bx = nodeCells(i,1); % bin index
        by = nodeCells(i,2);
        idNewTrue = trueWorld.bin2NodeID(by, bx);
        % add this new node to the exploredGraph
        if ( idNewTrue == 0 )
            % if this is a falsely declared node, place vertex randomly
            nodeX = trueWorld.xcp(bx) + (rand()-0.5)*trueWorld.binWidth;
            nodeY = trueWorld.ycp(by) + (rand()-0.5)*trueWorld.binWidth;
        else
            % otherwise place vertex according to the true world graph
            nodeX = trueWorld.nodeX(idNewTrue);
            nodeY = trueWorld.nodeY(idNewTrue);
        end
        % add node
        NodeProps = table(bx, by, nodeX, nodeY, 'VariableNames', {'bx', 'by', 'nodeX', 'nodeY'});
        swarmWorld.exploredGraph = addnode(swarmWorld.exploredGraph,NodeProps);
        % update bin to point to new node
        idNew = numnodes(swarmWorld.exploredGraph);
        swarmWorld.bin2NodeIDexplored(by,bx) = idNew;
        % add edges
        % check if any of the  8 cells around it have also been detected
        [bxNeighbor, byNeighbor] = getNeighborCells( bx, by, trueWorld.numBinsX, trueWorld.numBinsY );
        for k = 1:1:length(bxNeighbor)
            if ( swarmWorld.cellDetMat( byNeighbor(k) , bxNeighbor(k) ) == 1 )
                % to add edge, the neighbor cell must already be in the graph
                idNeighbor = swarmWorld.bin2NodeIDexplored( byNeighbor(k) , bxNeighbor(k) );
                if ( idNeighbor ~= 0 ) % neighbor is in graph
                    % if edge already exists, skip
                    if ( findedge(swarmWorld.exploredGraph,idNew,idNeighbor)==0 )
                        % check that indeed, the true world has this edge
                        idNeighborTrue = trueWorld.bin2NodeID(byNeighbor(k), bxNeighbor(k));
                        % if one of the nodes is zero, then this is a
                        % spurious node (false positive/not a real one)
                        % we add the edge to any nearby nodes
                        if ( idNewTrue == 0 || idNeighborTrue == 0 )
                            swarmWorld.exploredGraph = addedge(swarmWorld.exploredGraph,idNew,idNeighbor);
                            % otherwise we add edges according to true world
                        elseif ( trueWorld.A_env(idNewTrue,idNeighborTrue)==1 )
                            swarmWorld.exploredGraph = addedge(swarmWorld.exploredGraph,idNew,idNeighbor);
                            disp('edge added');
                            edgeDir = atan2( swarmWorld.exploredGraph.Nodes.nodeY(idNeighbor) - swarmWorld.exploredGraph.Nodes.nodeY(idNew), swarmWorld.exploredGraph.Nodes.nodeX(idNeighbor) - swarmWorld.exploredGraph.Nodes.nodeX(idNew));                            
                            if (edgeDir < 0)
                                edgeDir = pi+edgeDir;
                            end
                            swarmWorld.edgeDir = [swarmWorld.edgeDir; edgeDir];
                            
                            
                        else
                            disp('edge not added');
                        end
                        %                         % debug
                        %                         figure(1);
                        %                         imagesc(swarmWorld.cellDetMat,'XData',trueWorld.xcp,'YData',trueWorld.ycp);
                        %                         set(gca,'YDir','Normal')
                        %                         hold on;
                        %                         plot(nodeX,nodeY,'m+','MarkerSize',15);
                        %                             plot(trueWorld.nodeXY(idNeighborTrue,1), trueWorld.nodeXY(idNeighborTrue,2),'mo','MarkerSize',15);
                        %                         xmin = 0;
                        %                         ymin = 0;
                        %                         numXpx = floor(trueWorld.boxlength/trueWorld.binWidth);
                        %                         numYpx = floor(trueWorld.boxwidth/trueWorld.binWidth);
                        %                         drawgrid(xmin,numXpx,ymin,numYpx,trueWorld.binWidth)
                        %                         axis equal; axis tight; hold on;
                        %                         plot(trueWorld.G_env,'XData',trueWorld.nodeXY(:,1),'YData',trueWorld.nodeXY(:,2));
                        %                         plot(trueWorld.xx,trueWorld.yy,'co');
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