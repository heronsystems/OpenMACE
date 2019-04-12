function [runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_RandalsAtF3_LRDTtest(algorithmID,initialFormationID,targetMotionID)

% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runParams = struct;
runParams.type = 'matlab'; % 'matlab' 'mace' 'f3'
runParams.T =  30; %*60; % total simulation/mission time
runParams.dt = 0.01; % time-step (even if MACE is running, Sheng needs this for cost computation)

% F3 Flight Test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ROS_MACE = [];
if ( strcmp(runParams.type, 'mace') )
    ROS_MACE = struct;
    ROS_MACE.ip ='10.104.196.174'; % Artur Office
    %ROS_MACE.ip ='192.168.1.219'; % Artur Home
    ROS_MACE.operationalAlt = 5; % m
    ROS_MACE.xInit = 0; % position of first quad in local frame
    ROS_MACE.yInit = 0;
    ROS_MACE.initSpacing = 3; % spacing of additional quads
    ROS_MACE.coordSys = 'F3'; % 'ENU' or 'F3'
    % F3
    ROS_MACE.LatRef = 38.973699;
    ROS_MACE.LongRef = -76.921897;
    ROS_MACE.startOnPerimeter = 1; % 0/1 flag
end

% Display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runParams.flags.movie = 1;
runParams.movie.useBackgroundImg = 0; %
if ( runParams.movie.useBackgroundImg )
    runParams.movie.backgroundImgFile = './data/f3_bright.png';
    runParams.movie.backgroundImgBottomLeftCornerX = -73 + 2;
    runParams.movie.backgroundImgBottomLeftCornerY = -30;
    runParams.movie.backgroundImgWidth = 108;
    runParams.movie.backgroundImgHeight = 50;
    runParams.movie.plotBuffer = 0;
end
runParams.movie.plotF3Obstacles = 0;
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
swarmModel.N = 4; % number of agents
swarmModel.Rsense = 2.5; % sensing radius % 2 for F3 map % 20 for full map
swarmModel.vmax = 1.5; % maximum speed % 1 for F3 map % 20 for full map
swarmModel.umax = 2.0; % max acceleration
swarmModel.kp_wpt = 10.0; % agent waypoint control, proportional gain
swarmModel.kd_wpt = 5.0; % derivative gain
swarmModel.Tsamp = 0.5; % sample time

% agents follow a double integrator model with xdot = Ax + Bu and
% saturation on the input u. A, and B are defined below.
swarmModel.d = swarmModel.umax/swarmModel.vmax; % agent damping parameter
swarmModel.A = [0 0 1 0; 0 0 0 1; 0 0 -swarmModel.d 0; 0 0 0 -swarmModel.d];
swarmModel.B = [0 0; 0 0; 1 0; 0 1];
runParams.Tsamp = swarmModel.Tsamp; % make a copy

% Communication / task allocation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
swarmModel.communicationTopology = 'centralized';   % options are: 'centralized' or 'allToAll'
swarmModel.taskAllocation = 'stepwiseHungarian'; %'stepwiseHungarian'; % options are: 'none', 'stepwiseHungarian', 'Hungarian' or 'Auctioneer';
switch swarmModel.taskAllocation
    case 'Hungarian'
        swarmModel.samplesPerTask = 5;
    case 'stepwiseHungarian' % original
        swarmModel.samplesPerTask = 10;
        swarmModel.bundleSize = 4;
    case 'none'
        swarmModel.samplesPerTask = 5;
end

% Detection / Tracking (LRDT)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%swarmModel.Pd = 0.85;
%swarmModel.Pfa = 0.15;
swarmModel.sensorType = 'discrete_per_cell'; % options are : 'continuous_per_fov' or 'discrete_per_cell'
swarmModel.sensorDiscretizationLevels = 100;

swarmModel.LRDTOnTheFlyFlag = 1;
swarmModel.maxUnexploredPrior = 0.85;
swarmModel.probAbsentPrior = 0.25; % for initialization
swarmModel.probNullStateTransition = 0.1; % this term causes decay n CLR
%swarmModel.m = zTransform(swarmModel.Pd) - zTransform(swarmModel.Pfa); % for sensor model / msmt likelihood
swarmModel.m = 2;

swarmModel.terminateSimOnDetect = 1;
swarmModel.confLevel = 0.95;
swarmModel.cumlLRthresh = swarmModel.confLevel / (1 - swarmModel.confLevel); % initial value

% Monte carlo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% loading options: first input indicates the desired initial formation
%                  second input indicates the desired target motion

swarmModel.useGeneratedInitialFormation = 0;
swarmModel.useGeneratedTargetMotion = 0; % 0 will use a random target motion
monteCarloFlag = 0;
if nargin == 3
    monteCarloFlag = 1;
    swarmModel.algorithmID = algorithmID; % non-zero means you can try the following options
    %      1. lawnmower 2. random waypoints 3. mutualInfo waypoints + Hungarian 4. mutualInfo + stepwiseHungarian
    
    swarmModel.useGeneratedInitialFormation = 1; % 1 will load pre-randomly generated initial formation
    swarmModel.initialFormationID = initialFormationID; % load the ID of the initial formation
    
    swarmModel.useGeneratedTargetMotion = 1; % 1 will load pre-randomly generated target motion
    swarmModel.targetMotionID = targetMotionID; % load the ID of the initial formation
    
    switch swarmModel.algorithmID
        case 1
            swarmModel.taskGeneration = 'lawnmower';
            swarmModel.taskAllocation = 'none';
            swarmModel.m = 1;
        case 2
%             swarmModel.taskGeneration = 'randomWpts';
%             swarmModel.taskAllocation = 'none';
            swarmModel.taskGeneration = 'lawnmower';
            swarmModel.taskAllocation = 'none';
            swarmModel.m = 2;
        case 3
            swarmModel.taskGeneration = 'lawnmower';
            swarmModel.taskAllocation = 'none';
            swarmModel.m = 3;
        case 4
            swarmModel.taskGeneration = 'mutualInfoWpts'; % Hungarian
            swarmModel.taskAllocation = 'stepwiseHungarian';
            swarmModel.m = 1;
        case 5
            swarmModel.taskGeneration = 'mutualInfoWpts'; % Hungarian
            swarmModel.taskAllocation = 'stepwiseHungarian';
            swarmModel.m = 2;
        case 6
            swarmModel.taskGeneration = 'mutualInfoWpts'; % Hungarian
            swarmModel.taskAllocation = 'stepwiseHungarian';            
            swarmModel.m = 3;
        case 7
            swarmModel.taskGeneration = 'likelihoodWpts'; % Hungarian
            swarmModel.taskAllocation = 'stepwiseHungarian';
            swarmModel.m = 1;
        case 8
            swarmModel.taskGeneration = 'likelihoodWpts'; % Hungarian
            swarmModel.taskAllocation = 'stepwiseHungarian';
            swarmModel.m = 2;
        case 9
            swarmModel.taskGeneration = 'likelihoodWpts'; % Hungarian
            swarmModel.taskAllocation = 'stepwiseHungarian';            
            swarmModel.m = 3;            
    end
    fprintf('Monte carlo settings: %s \n',swarmModel.taskGeneration)
else
    fprintf('Monte carlo settings not used\n');
end

% Task Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ( ~monteCarloFlag ) % if running a single-run, use this value:
    swarmModel.taskGeneration = 'lawnmower'; % 'randomWpts', or 'frontierWpts', 'lawnmower' 'mutualInfoWpts'
end
switch swarmModel.taskGeneration
    case 'frontierWpts'
        swarmModel.wptChangePeriod = 10; % sec, how often we generate a new random wpt
        swarmModel.mapping.method = 'frontierAndBlob'; % options are: 'frontierOnly' or 'frontierAndBlob'
        swarmModel.mapping.minBlobArea = pi*swarmModel.Rsense^2*2;  % threshold for the minimum area of a blob
        swarmModel.mapping.maxMajorMinorAxisRatio = 10; % threshold for the ratio between majoraxislength and minor axis length of a subblob, give inf if you do not want to apply this filter
        swarmModel.mapping.blobCostScale = 1.4; % scale the cost for reaching a subblob centroid by this amount
    case 'mutualInfoWpts'
        swarmModel.numTasks = 200;
        swarmModel.stepSizeGain = 0.2;
        swarmModel.percentTol = 0.03;
        swarmModel.maxIters = 500;
    case 'likelihoodWpts'
        swarmModel.numTasks = 200;
        swarmModel.stepSizeGain = 0.2;
        swarmModel.percentTol = 0.03;
        swarmModel.maxIters = 500;        
end

swarmModel.mapping.krigingSigma = 3; % controls how much kriging interp diffuses
swarmModel.utilityComputation = 'computeInformationGain'; % options are: 'computeEnergyAndPenalty' or 'computeInformationGain'
swarmModel.planningHorizon = swarmModel.samplesPerTask * swarmModel.Tsamp; %runParams.T; %

% Target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
targetModel = struct;
targetModel.M = 1; % number of targets
targetModel.type = 'constantSpeedRandomWalk'; % 'varyingSpeedRandomWalk' or 'constantSpeedRandomWalk'
targetModel.probStopping = 0.5;
targetModel.m = 1.0;
targetModel.d = 0.1;
switch targetModel.type
    case 'varyingSpeedRandomWalk'
        targetModel.maxSpeed = 10;
    case 'constantSpeedRandomWalk'
        targetModel.restPeriod = swarmModel.Tsamp;
        targetModel.inertia = 100; % a value greater than zero
        if (monteCarloFlag)
            if swarmModel.useGeneratedTargetMotion
                load(['./scenes/targetMotion' num2str(swarmModel.targetMotionID) '.mat']);  % load the generated data to replace the above parameters.
            end
        end
end

% Environment/Map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % load environment cityblocks
% trueWorld = struct;
% trueWorld.type = 'cityblocks'; % 'goxel', 'cityblocks', %'openStreetMap', 'osmAtF3', 'cityBlocksAtF3'
% trueWorld.borderOffset = 10; % used for adding padding to the map
% trueWorld.binWidth = 5; % distance used to declare two nodes as connected (use 7 for open street map)
% trueWorld.folder = './data/'; % folder with map file
% trueWorld.fileName = 'cityBlocks';
% trueWorld.blockLength = 50;
% trueWorld.numBlocks = 4;

% load environment smaller map at F3
trueWorld = struct;
trueWorld.type = 'osmAtF3'; % 'goxel', 'cityblocks', %'openStreetMap', 'osmAtF3'
trueWorld.borderOffset = 0; % used for adding padding to the map
trueWorld.folder = './data/'; % folder with map file
trueWorld.binWidth=1;
trueWorld.fileName = 'RandallsIsland_Big.osm';
trueWorld.f3Workspace = 'full'; % 'full', 'right-square'
trueWorld.refX = -350;
trueWorld.refY = 80;
trueWorld.angle = -37*pi/180;
trueWorld.boxlength = 500;
trueWorld.buffer = 1;

% load environment the full map
% trueWorld = struct;
% trueWorld.type = 'openStreetMap'; % 'cityblocks', %'openStreetMap', 'osmAtF3'
% trueWorld.borderOffset = 0; % used for adding padding to the map
% %trueWorld.binWidth = 1; % distance used to declare two nodes as connected (use 7 for open street map)
% trueWorld.folder = './data/'; % folder with map file
% trueWorld.fileName = 'RandallsIsland_Big.osm';
% trueWorld.binWidth = 5;
% trueWorld.refX = -300;
% trueWorld.refY = -200;
% trueWorld.angle = 0*pi/180;
% trueWorld.boxlength = 400;
% trueWorld.boxwidth = 400;
% trueWorld.buffer = 0;

% derived world model parameters
trueWorld = loadEnvironment(trueWorld, targetModel);

% given the sensing radius, measurements can only be obtained within some
% window of bins around the target
trueWorld.windowWidth = 3*ceil(swarmModel.Rsense/trueWorld.binWidth)+1;
trueWorld.halfWidth = floor((trueWorld.windowWidth-1)/2);

% Misc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ( strcmp(runParams.type, 'matlab') )
    runParams.Nsim = floor(runParams.T/runParams.dt)+1; % number of time-steps to simulate
elseif ( strcmp(runParams.type, 'mace') )
    % origin in utm coordinates
    [ROS_MACE.XRef,ROS_MACE.YRef,ROS_MACE.utmzone] = deg2utm(ROS_MACE.LatRef,ROS_MACE.LongRef);
    ROS_MACE.N = swarmModel.N; % make copy
    % initial position of quads is along a line according to initSpacing
    xInit = linspace(ROS_MACE.xInit,ROS_MACE.xInit+ROS_MACE.initSpacing*ROS_MACE.N,ROS_MACE.N);
    yInit = ones(1,ROS_MACE.N)*ROS_MACE.yInit;
    % create arducopter script with these initial conditions
    generateArducopterCmds( ROS_MACE , xInit , yInit );
end




end
