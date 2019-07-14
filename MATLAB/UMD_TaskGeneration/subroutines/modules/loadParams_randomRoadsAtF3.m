function [runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_randomRoadsAtF3()

% simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runParams = struct;
runParams.type = 'matlab'; % 'matlab' 'mace' 'f3'
runParams.T = 1*60; % total simulation/mission time


% F3 Flight Test
ROS_MACE=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ( strcmp(runParams.type, 'mace') )
    ROS_MACE = struct;
    ROS_MACE.operationalAlt = [3 5]; % m OR [4 8 2 6]; if running four quads
    ROS_MACE.agentIDs = [1 2]; % m OR [1 2 3 4]; if running four quads
    ROS_MACE.agentIDtoIndex = zeros(1,max(ROS_MACE.agentIDs));
    for i = 1:1:length(ROS_MACE.agentIDs)
        ROS_MACE.agentIDtoIndex( ROS_MACE.agentIDs(i) ) = i;
    end
    
    %ROS_MACE.ip ='10.104.193.156'; % Artur Office
    ROS_MACE.ip ='127.0.0.1';
    %ROS_MACE.ip ='192.168.1.219'; % Artur Home    
    ROS_MACE.xInit = 0; % position of first quad in local frame
    ROS_MACE.yInit = 0;
    ROS_MACE.initSpacing = 3;
    ROS_MACE.coordSys = 'F3'; % 'ENU' or 'F3'
    % F3
    ROS_MACE.LatRef = 38.973699;
    ROS_MACE.LongRef = -76.921897;
    ROS_MACE.startOnPerimeter = 1; % 0/1 flag
    ROS_MACE.trails = 10; % length of visual trail, in no. samples
    
    ROS_MACE.wptCoordinator = 'standalone'; % options are 'integrated' and 'standalone'
end

runParams.dt = 0.1; % time-step (even if MACE is running, Sheng needs this for cost computation)

% Display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runParams.flags.movie = 1; % play movie
runParams.movie.useBackgroundImg = 1; %
if ( runParams.movie.useBackgroundImg )
    runParams.movie.backgroundImgFile = './data/f3_bright.png';
    runParams.movie.backgroundImgBottomLeftCornerX = -73 + 2;
    runParams.movie.backgroundImgBottomLeftCornerY = -30;
    runParams.movie.backgroundImgWidth = 108;
    runParams.movie.backgroundImgHeight = 50;
    runParams.movie.plotBuffer = 5;
end

runParams.movie.plotF3Obstacles = 1;
if ( runParams.movie.plotF3Obstacles )
    runParams.movie.perimX = [-59 28 28 -59 -59];
    runParams.movie.perimY = [13 13 -13 -13 13];
    R = 2;
    numPts = 20;
    [runParams.movie.pole1x, runParams.movie.pole1y] = generateCircle(0, 0, R, numPts);
    [runParams.movie.pole2x, runParams.movie.pole2y] = generateCircle(-30, 0, R, numPts);
end

% Swarm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
swarmModel = struct;
swarmModel.N = 2; % number of agents OR 4; if running four quads
swarmModel.Rsense = 1.5; % sensing radius

%swarmModel.delay = 1.3564;
swarmModel.vmax = 1.2145; % maximum speed
swarmModel.umax = 0.3478; % max acceleration
swarmModel.kp_wpt = 2.0816; % agent waypoint control, proportional gain
swarmModel.kd_wpt = 13.9315; % derivative gain
swarmModel.Tsamp = 2; % sample time

% agents follow a double integrator model with xdot = Ax + Bu and
% saturation on the input u. A, and B are defined below.
swarmModel.d = swarmModel.umax/swarmModel.vmax; % agent damping parameter
swarmModel.A = [0 0 1 0; 0 0 0 1; 0 0 -swarmModel.d 0; 0 0 0 -swarmModel.d];
swarmModel.B = [0 0; 0 0; 1 0; 0 1];
runParams.Tsamp = swarmModel.Tsamp; % make a copy

%swarmModel.samplesPerTask = 2; 
%swarmModel.taskGeneration = 'frontierWpts'; % 'randomWpts', or 'frontierWpts' 
%swarmModel.Told = 60; % sec, time used for saturated pixel "age".
% Note, even if MACE is running we need these for prediction (i.e., Sheng's
% cost function)

% Communication / task allocation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
swarmModel.communicationTopology = 'centralized';   % options are: 'centralized' or 'allToAll'
if ( nargin ~=3 ) % if running a single-run, use this value:
    swarmModel.taskAllocation = 'stepwiseHungarian_unique'; %'stepwiseHungarian'; % options are: 'none', 'stepwiseHungarian', 'Hungarian' or 'Auctioneer';
end
switch swarmModel.taskAllocation
    case 'Hungarian'
        swarmModel.samplesPerTask = 5;
    case 'stepwiseHungarian' % original
        swarmModel.samplesPerTask = 10;
        swarmModel.bundleSize = 5;
        swarmModel.neighborMethod = 'knn';  % options are: 'VoronoiGraph' or 'knn'
    case 'stepwiseHungarian_unique' % original
        swarmModel.samplesPerTask = 10;
        swarmModel.bundleSize = 5;
        swarmModel.neighborMethod = 'knn';  
        swarmModel.knnNumber = 15;
    case 'stepwiseHungarian_max' % original
        swarmModel.samplesPerTask = 10;
        swarmModel.bundleSize = 4;
    case 'stepwiseHungarian_2ndOrder' % original
        swarmModel.samplesPerTask = 10;
        swarmModel.bundleSize = 4;
    
    case 'none'
        swarmModel.samplesPerTask = 5;
end

% Task Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ( nargin ~=3 ) % if running a single-run, use this value:
    swarmModel.taskGeneration = 'mutualInfoWpts'; % 'randomWpts', or 'frontierWpts', 'lawnmower' 'mutualInfoWpts'
end
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
        swarmModel.percentTol = 0.05;
        swarmModel.maxIters = 250;
    case 'likelihoodWpts'
        swarmModel.numTasks = 200;
        swarmModel.stepSizeGain = 0.2;
        swarmModel.percentTol = 0.03;
        swarmModel.maxIters = 500;
end

swarmModel.mapping.krigingSigma = 0.5; % controls how much kriging interp diffuses
swarmModel.utilityComputation = 'computeInformationGain'; % options are: 'computeEnergyAndPenalty' or 'computeInformationGain'
swarmModel.planningHorizon = swarmModel.samplesPerTask * swarmModel.Tsamp; %runParams.T; %

% Mapping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
swarmModel.mappingSensorType = 'noisy'; % 'noisy' or 'perfect'
if ( strcmp(swarmModel.mappingSensorType,'noisy') )
    if ( nargin ~=3 )
        swarmModel.mG = 4; % sensitivity
    end
    swarmModel.nG = 100; % number of discrete sensor levels
    swarmModel.mapConfLevel = 0.95;
    swarmModel.nodeLRthresh = swarmModel.mapConfLevel / (1 - swarmModel.mapConfLevel); % initial value
    % derived
    swarmModel.gval = linspace(-3,swarmModel.mG+3,swarmModel.nG);
    for l = 1:1:swarmModel.nG
        swarmModel.g_V(l) = exp(-(swarmModel.gval(l))^2/2);
        swarmModel.g_UO(l) = exp(-(swarmModel.gval(l)-swarmModel.mG)^2/2);
    end
    swarmModel.g_V = swarmModel.g_V ./ sum(swarmModel.g_V);
    swarmModel.g_UO = swarmModel.g_UO ./ sum(swarmModel.g_UO);
end

% Detection / Tracking (LRDT)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%swarmModel.Pd = 0.85;
%swarmModel.Pfa = 0.15;
swarmModel.sensorType = 'discrete_per_cell'; % options are : 'continuous_per_fov' or 'discrete_per_cell'
swarmModel.LRDTOnTheFlyFlag = 1;
%swarmModel.maxUnexploredPrior = 0.85;
swarmModel.nodeDensityInitGuess = 1/3; % used on first step before kriging takes place
swarmModel.probAbsentPrior = 0.50; % for initialization
swarmModel.numNodesEstPercent = 1/5; 

swarmModel.decayRate = 0.05; % value from 0 to 1
swarmModel.q_s_n =  swarmModel.decayRate*(1-swarmModel.probAbsentPrior);
swarmModel.q_n_s =  swarmModel.decayRate*swarmModel.probAbsentPrior;

swarmModel.LReqbm = swarmModel.q_s_n / swarmModel.q_n_s; % note LR reaches eqbm value equal to q_s_n/q_n_s
swarmModel.q_n_n = 1 - swarmModel.q_s_n;


%swarmModel.m = zTransform(swarmModel.Pd) - zTransform(swarmModel.Pfa); % for sensor model / msmt likelihood
swarmModel.terminateSimOnDetect = 0;
swarmModel.confLevel = 0.95;
if ( nargin ~=3 )
    swarmModel.mZ = 3;
end
swarmModel.nZ = 100;

% derived
swarmModel.cumlLRthresh = swarmModel.confLevel / (1 - swarmModel.confLevel); % initial value
swarmModel.zval = linspace(-3,swarmModel.mZ+3,swarmModel.nZ);
for q = 1:1:swarmModel.nZ
    swarmModel.z_VU(q) = exp(-(swarmModel.zval(q))^2/2);
    swarmModel.z_O(q) = exp(-(swarmModel.zval(q)-swarmModel.mZ)^2/2);
end
swarmModel.z_VU = swarmModel.z_VU ./ sum(swarmModel.z_VU);
swarmModel.z_O = swarmModel.z_O ./ sum(swarmModel.z_O);

% Target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
targetModel = struct;
targetModel.M = 1; % number of targets
targetModel.type = 'constantSpeedRandomWalk'; % 'varyingSpeedRandomWalk' or 'constantSpeedRandomWalk'
targetModel.probStopping = 0.75;
targetModel.m = 1.0;
targetModel.d = 0.1;
switch targetModel.type
    case 'varyingSpeedRandomWalk'
        targetModel.maxSpeed = 10;
    case 'constantSpeedRandomWalk'
        targetModel.restPeriod = swarmModel.Tsamp;
        targetModel.inertia = 100; % a value greater than zero
%         if (monteCarloFlag)
%             if swarmModel.useGeneratedTargetMotion
%                 load(['./scenes/targetMotion' num2str(swarmModel.targetMotionID) '.mat']);  % load the generated data to replace the above parameters.
%             end
%         end
end

% Environment/Map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load environment 
trueWorld = struct;
trueWorld.type = 'randomRoadsAtF3'; % 'cityblocks', %'openStreetMap', 'osmAtF3'
trueWorld.f3Workspace = 'right-square'; % 'full', 'right-square'
trueWorld.borderOffset = 0; % used for adding padding to the map
trueWorld.binWidth = 0.5; % distance used to declare two nodes as connected (use 7 for open street map)
trueWorld.folder = './data/'; % folder with map file
% trueWorld.boxlength = 400;
% trueWorld.boxwidth = 400;
trueWorld.buffer = 0;
trueWorld.fileName = 'randomRoadsAtF3';
trueWorld.nodeFile = './external/road-network/node-list';
trueWorld.edgeFile = './external/road-network/edge-list';

% derived world model parameters
trueWorld = loadEnvironment(trueWorld, targetModel);

% given the sensing radius, measurements can only be obtained within some
% window of bins around the target
trueWorld.windowWidth = 3*ceil(swarmModel.Rsense/trueWorld.binWidth)+1;
trueWorld.halfWidth = floor((trueWorld.windowWidth-1)/2);

% Misc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
swarmModel.useGeneratedInitialFormation = 0;
if ( strcmp(runParams.type, 'matlab') )
    runParams.Nsim = floor(runParams.T/runParams.dt)+1; % number of time-steps to simulate
elseif ( strcmp(runParams.type, 'mace') )
    % origin in utm coordinates
    [ROS_MACE.XRef,ROS_MACE.YRef,ROS_MACE.utmzone] = deg2utm(ROS_MACE.LatRef,ROS_MACE.LongRef);
    ROS_MACE.N = swarmModel.N; % make copy
    ROS_MACE.tempHandle = cell(2,ROS_MACE.N);
    % initial position of quads is along a line according to initSpacing
    xInit = linspace(ROS_MACE.xInit,ROS_MACE.xInit+ROS_MACE.initSpacing*ROS_MACE.N,ROS_MACE.N);
    yInit = ones(1,ROS_MACE.N)*ROS_MACE.yInit;
    % create arducopter script with these initial conditions
    generateArducopterCmds( ROS_MACE , xInit , yInit );
    
    diary off;
    diaryFileName = ['F3DiaryLog_' datestr(now,'dd_mmm_yyyy_HHMMSS') '.txt'];
    diary(diaryFileName);
end
end
