function [swarmState, swarmWorld] = taskAllocation(tasks, swarmState, swarmModel, swarmWorld, trueWorld, simParams)
% This function should produce a desired waypoint or desired waypoint list
% for each agent
% taskManagment assigns the waypoints in the list

if strcmp(swarmModel.taskAllocation,'stepwiseHungarian_unique')
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
            idx = knnsearch(swarmWorld.cellCenterOfMass,reshape(swarmState.x(4*(kk-1)+1:4*(kk-1)+2),1,2),'K',swarmModel.knnNumber+1);
            % we might need to unify the representation of swarmState.x as
            % a column or row vector for the above line
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
    end
end
