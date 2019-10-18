function swarmWorld = initializeSwarmWorld(trueWorld , swarmModel, runParams)

% swarm view of the state of the world.
swarmWorld = struct;

% cellStateMat has entries that indicate:
% 0 - an explored cell found to be empty
% 1 - an explored cell found to contain nodes
% 2 - an unexplored cell
swarmWorld.cellStateMat = ones(size(trueWorld.bin2NodeID))*2;

% number of measurements in each cell
%swarmWorld.cellMsmtMat = zeros(size(trueWorld.bin2NodeID));

% if cell is detected node (1) unknown (0) detected void (-1)
swarmWorld.cellDetMat  = zeros(size(trueWorld.bin2NodeID));
swarmWorld.bin2NodeIDexplored = zeros(size(trueWorld.bin2NodeID));

% forecast corresponds to a linear (kriging) interpolation 
swarmWorld.forecastMat = zeros(size(trueWorld.bin2NodeID));

% timeElapsedMat indicates the age of each cell 
swarmWorld.timeElapsedMat = ones(size(trueWorld.bin2NodeID))*runParams.T;

% additional values stored by the world model
swarmWorld.time = 0; 
swarmWorld.exploredGraph = graph;
swarmWorld.targetStateSpaceGraph = digraph;
swarmWorld.Mpc2s = [];
swarmWorld.Mc = [];
swarmWorld.Mp = [];
swarmWorld.Q = [];
swarmWorld.mapPercentage = [];
swarmWorld.edgeDir = [];
swarmWorld.numViews = 0;

% informed prior
swarmWorld.nodeDensity = swarmModel.nodeDensityInitGuess;

vNom = (1 - swarmWorld.nodeDensity);
oNom = (1-vNom)*(1 - swarmModel.probAbsentPrior)/(trueWorld.numBinsX*trueWorld.numBinsY);
uNom = 1 - vNom - oNom;
swarmWorld.V = vNom*ones(size(trueWorld.bin2NodeID));
swarmWorld.U = uNom*ones(size(trueWorld.bin2NodeID));
swarmWorld.O = oNom*ones(size(trueWorld.bin2NodeID));
%swarmWorld.samplingPriority = samplingPriority( swarmWorld );

swarmWorld.mutualInformation = ones(size(trueWorld.bin2NodeID));
swarmWorld.log_likelihood = [];
swarmWorld.log_likelihood_env = [];
swarmWorld.targetDetectedFlag = 0;

end
