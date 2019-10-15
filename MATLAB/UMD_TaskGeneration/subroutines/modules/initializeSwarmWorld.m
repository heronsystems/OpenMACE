function swarmWorld = initializeSwarmWorld(trueWorld , swarmModel, runParams)
% Sheng: this is where allToAllCommunication starts
switch swarmModel.communicationTopology
    case 'centralized'
		% swarm view of the state of the world.
		swarmWorld = struct;
		
		% cellStateMat has entries that indicate:
		% 0 - an explored cell found to be empty
		% 1 - an explored cell found to contain nodes
		% 2 - an unexplored cell
		swarmWorld.cellStateMat = ones(size(trueWorld.numNodesMat))*2;
		
        % number of measurements in each cell
        swarmWorld.cellMsmtMat = zeros(size(trueWorld.numNodesMat));
        
        % if cell is detected node (1) unknown (0) detected void (-1)
        swarmWorld.cellDetMat  = zeros(size(trueWorld.numNodesMat));
        swarmWorld.bin2NodeIDexplored = zeros(size(trueWorld.numNodesMat));

		% forecast corresponds to a linear (kriging) interpolation of the
		% cellStateMat using only measurements where entries are == 1 (where nodes
		% were found)
		swarmWorld.forecastMat = zeros(size(trueWorld.numNodesMat));
		
		% timeElapsedMat indicates the age of each cell (time since it was last
		% viewed) saturated by the value Told
		swarmWorld.timeElapsedMat = ones(size(trueWorld.numNodesMat))*runParams.T;
		
		% additional values stored by the world model
		swarmWorld.time = 0; % time at current swarm world state
		
		% initialize the explored map
		swarmWorld.exploredGraph = graph;
		% assign a larger node index to the 'dark' node (unexplored nodes)
		% swarmWorld.exploredGraph = addnode(swarmWorld.exploredGraph,1);
		% initilize attributes to nodes
		% swarmWorld.exploredGraph.Nodes.trueGraphIndex = [(length(trueWorld.nodeX)+1)];
		
		% initialize a the frontier index (Nfrontier x 1) vector containing
		% the index of the frontier nodes
		swarmWorld.frontierIndex = [];
		
		% initialize a vector containing the coordinate of centroid of subblobs 
		swarmWorld.subblobCentroid = [];
		
		% make the simulation deterministic
		% rng(10)
		
		% explored targetStateSpace
		swarmWorld.targetStateSpaceGraph = digraph;
		swarmWorld.Mpc2s = [];
		swarmWorld.Mc = [];
		swarmWorld.Mp = [];
		swarmWorld.Q = [];
		
		% Sheng: add a field 'mapPercentage' under swarmWorld to evaluate the percentage
		% of nodes explored
		swarmWorld.mapPercentage = [];
		swarmWorld.numViews = 0;
		
		% uniform prior
%         vNom = 1/3;
%         uNom = 1/3;
%         oNom = 1/3;
        
        % informed prior
        swarmWorld.nodeDensity = swarmModel.nodeDensityInitGuess;
        vNom = (1 - swarmWorld.nodeDensity);
        oNom = (1-vNom)*(1 - swarmModel.probAbsentPrior)/(trueWorld.numBinsX*trueWorld.numBinsY);
        uNom = 1 - vNom - oNom;
        
        swarmWorld.V = vNom*ones(size(trueWorld.numNodesMat));
        swarmWorld.U = uNom*ones(size(trueWorld.numNodesMat));
        swarmWorld.O = oNom*ones(size(trueWorld.numNodesMat));

        swarmWorld.samplingPriority = samplingPriority( swarmWorld );
        
        swarmWorld.mutualInformation = ones(size(trueWorld.numNodesMat));

        % assume that prob target is not present is 0.75
        if ( swarmModel.LRDTOnTheFlyFlag )
            swarmWorld.log_likelihood = [];
            swarmWorld.log_likelihood_env = [];
        else
            swarmModel.probPresent = (1-swarmModel.probAbsent)*ones(trueWorld.Ns,1)/trueWorld.Ns;
            swarmWorld.log_likelihood = log( swarmModel.probPresent ./ swarmModel.probAbsent );
            swarmWorld.log_likelihood_env = projectLikelihood( swarmWorld.log_likelihood , trueWorld.Mc );
        end
        swarmWorld.targetDetectedFlag = 0;
        
    case 'allToAll'
        for k = 1:swarmModel.N
			% swarm view of the state of the world.
			swarmWorld{k} = struct;
			
			% cellStateMat has entries that indicate:
			% 0 - an explored cell found to be empty
			% 1 - an explored cell found to contain nodes
			% 2 - an unexplored cell
			swarmWorld{k}.cellStateMat = ones(size(trueWorld.numNodesMat))*2;
			
			% forecast corresponds to a linear (kriging) interpolation of the
			% cellStateMat using only measurements where entries are == 1 (where nodes
			% were found)
			swarmWorld{k}.forecastMat = zeros(size(trueWorld.numNodesMat));
			
			% timeElapsedMat indicates the age of each cell (time since it was last
			% viewed) saturated by the value Told
			%swarmWorld{k}.timeElapsedMat = ones(size(trueWorld.numNodesMat))*runParams.T;
			
			% additional values stored by the world model
			swarmWorld{k}.time = 0; % time at current swarm world state
			
			% initialize the explored map
			swarmWorld{k}.exploredGraph = graph;
			% assign a larger node index to the 'dark' node (unexplored nodes)
			% swarmWorld.exploredGraph = addnode(swarmWorld.exploredGraph,1);
			% initilize attributes to nodes
			% swarmWorld.exploredGraph.Nodes.trueGraphIndex = [(length(trueWorld.nodeX)+1)];
			
			% initialize a the frontier index (Nfrontier x 1) vector containing
			% the index of the frontier nodes
			swarmWorld{k}.frontierIndex = [];
			
			% initialize a vector containing the coordinate of centroid of subblobs 
			swarmWorld{k}.subblobCentroid = [];
			
            % initialize a matrix to contain the state of other agents (N)
            % The i-th column stores the state of the i-th agent
            swarmWorld{k}.allAgentState = nan(4,swarmModel.N);
            
			% make the simulation deterministic
			% rng(10)
			
			% explored targetStateSpace
			swarmWorld{k}.targetStateSpaceGraph = digraph;
			swarmWorld{k}.Mpc2s = [];
			swarmWorld{k}.Mc = [];
			swarmWorld{k}.Mp = [];
			swarmWorld{k}.Q = [];
			
			% Sheng: add a field 'mapPercentage' under swarmWorld to evaluate the percentage
			% of nodes explored
			swarmWorld{k}.mapPercentage = [];
			
            % initialize the communication matrix
            swarmWorld{k}.communicationMat = zeros(swarmModel.N);
            
            % initialize a list of every task (for numbering each task with a unique ID)
            swarmWorld{k}.everyTask = [];
            
            % initialize a placeholder for the availability of the
            % corresponding tasks
            swarmWorld{k}.availableTask = [];
            
            % initialize a placeholder of all the remaining tasks (the first one in the list is being pursued)
            swarmWorld{k}.currentTaskList = [];
            
            % initialize a placeholder of the assigned bundle
            swarmWorld{k}.assignedBundle = [];
            
            % the following block only defines an all-to-all communication
            % topology
            for i = 1:swarmModel.N-1
                for j = i+1:swarmModel.N
                    swarmWorld{k}.communicationMat(i,j) = 1;
                end
            end
            
			% graph of the target state space
			
			
			% assume that prob target is not present is 0.75
			if ( swarmModel.LRDTOnTheFlyFlag )
				%swarmModel.probPresent = (1-swarmModel.probAbsent)/trueWorld.Ns;
				swarmWorld{k}.log_likelihood = [];
				swarmWorld{k}.log_likelihood_env = [];
			else
				swarmModel.probPresent = (1-swarmModel.probAbsent)*ones(trueWorld.Ns,1)/trueWorld.Ns;
				swarmWorld{k}.log_likelihood = log( swarmModel.probPresent ./ swarmModel.probAbsent );
				swarmWorld{k}.log_likelihood_env = projectLikelihood( swarmWorld{k}.log_likelihood , trueWorld.Mc );
			end
			swarmModel.m = zTransform(swarmModel.Pd) - zTransform(swarmModel.Pfa);
        end
end
