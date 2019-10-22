function [runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_cityBlocks()

% simulation
runParams = struct;
runParams.type = 'matlab'; % 'matlab' 'mace' 'f3'
runParams.T = 10*1; % total simulation/mission time
runParams.dt = 0.1; % time-step 

runParams.flags.movie = 1; % play movie
runParams.movie.plotF3Obstacles = 0;
% load environment
trueWorld = struct;
trueWorld.type = 'cityblocks'; % 'cityblocks', %'openStreetMap', 'osmAtF3'
trueWorld.borderOffset = 0; % used for adding padding to the map
trueWorld.folder = './data/'; % folder with map file
trueWorld.binWidth = 1;
trueWorld.buffer = 0;
trueWorld.fileName = 'cityblocks';
trueWorld.blockLength = 10;
trueWorld.numBlocks = 8;
ROS_MACE=[];
runParams.movie.useBackgroundImg = 0;

% search agent properties
swarmModel = struct;
swarmModel.N = 4; % number of agents
swarmModel.Rsense = 3; % sensing radius
swarmModel.Tsamp = 1; % sample time
swarmModel.samplesPerTask = 5; 
swarmModel.taskGeneration = 'randomWpts'; % 'mutualInfoWpts', 'randomWpts', or 'frontierWpts' 
% Note, even if MACE is running we need these for prediction (i.e., Sheng's
% cost function)
swarmModel.vmax = 2; % maximum speed
swarmModel.umax = 2; % max acceleration
swarmModel.kp_wpt = 10.0; % agent waypoint control, proportional gain
swarmModel.kd_wpt = 5.0; % derivative gain
switch swarmModel.taskGeneration
    case 'frontierWpts'
        swarmModel.wptChangePeriod = 10; % sec, how often we generate a new random wpt
        swarmModel.mapping.method = 'frontierAndBlob'; % options are: 'frontierOnly' or 'frontierAndBlob'
        swarmModel.mapping.minBlobArea = pi*swarmModel.Rsense^2*2;  % threshold for the minimum area of a blob
        swarmModel.mapping.maxMajorMinorAxisRatio = 10; % threshold for the ratio between majoraxislength and minor axis length of a subblob, give inf if you do not want to apply this filter
        swarmModel.mapping.blobCostScale = 1.4; % scale the cost for reaching a subblob centroid by this amount
    case 'mutualInfoWpts'
        swarmModel.numTasks = 100;
        swarmModel.stepSizeGain = 0.2;
        swarmModel.percentTol = 0.03;
        swarmModel.maxIters = 500;        
end
swarmModel.mapping.krigingSigma = 2; % controls how much kriging interp diffuses
swarmModel.mapping.krigingMethod = 'recursive'; % options are: 'recursive' or 'standard'
swarmModel.communicationTopology = 'centralized';   % options are: 'centralized' or 'allToAll'
swarmModel.taskAllocation = 'Hungarian'; % options are: 'Hungarian' or 'Auctioneer';
swarmModel.utilityComputation = 'computeInformationGain'; % options are: 'computeEnergyAndPenalty' or 'computeInformationGain'
swarmModel.planningHorizon = runParams.T; %2*swarmModel.samplesPerTask * swarmModel.Tsamp;

% target properties
targetModel = struct;
targetModel.M = 1; % number of targets
targetModel.type = 'constantSpeedRandomWalk'; % 'varyingSpeedRandomWalk' or 'constantSpeedRandomWalk'
targetModel.probStopping = 0.50;
targetModel.m = 0;
targetModel.d = 0.1;
switch targetModel.type
    case 'varyingSpeedRandomWalk'
        targetModel.maxSpeed = 10;
    case 'constantSpeedRandomWalk'
        targetModel.restPeriod = swarmModel.Tsamp;
        targetModel.inertia = 100; % a value greater than zero
end

% tracking
swarmModel.LRDTOnTheFlyFlag = 1;
swarmModel.Pd = 0.95;
swarmModel.Pfa = 0.10;
swarmModel.positionSensorStDev = 1;
swarmModel.decayFactor = 0.0;

% voronoi mutual information
swarmModel.mutualInfoSurfaceBlurFlag = 0;
swarmModel.maxUnexploredPrior = 0.85;
swarmModel.sensorDiscretizationLevels = 100;

% for initialization
swarmModel.probAbsent = 0.75;

% computed derived parameters, using the above inputs
[runParams, swarmModel, trueWorld, ROS_MACE] = computeDerivedParams(runParams, swarmModel, trueWorld, ROS_MACE, targetModel);

end
