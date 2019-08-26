function [runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_cityblocksAtF3(mapID,algorithmID,initialFormationID,targetMotionID)

% Simulation
runParams = struct;
runParams.type = 'mace'; % 'matlab' 'mace' 'f3'
runParams.T = 8*60; % total simulation/mission time
runParams.dt = 0.01; % time-step (even if MACE is running used for prediction)
runParams.soundFlag = 1;

% F3 Flight Test
ROS_MACE = [];
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

% Display
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
    R = 1.5;
    numPts = 20;
    [runParams.movie.pole1x, runParams.movie.pole1y] = generateCircle(0, 0, R, numPts);
    [runParams.movie.pole2x, runParams.movie.pole2y] = generateCircle(-30, 0, R, numPts);
end

% Swarm
swarmModel = struct;
swarmModel.N = 2; % number of agents OR 4; if running four quads
swarmModel.Rsense = 1.5; % sensing radius

%swarmModel.delay = 1.3564;
swarmModel.vmax = 1.2145; % maximum speed
swarmModel.umax = 0.3478; % max acceleration
swarmModel.kp_wpt = 2.0816; % agent waypoint control, proportional gain
swarmModel.kd_wpt = 13.9315; % derivative gain
swarmModel.Tsamp = 2; % sample time

% Monte carlo
swarmModel.useGeneratedInitialFormation = 0;
swarmModel.useGeneratedTargetMotion = 0; % 0 will use a random target motion
%monteCarloFlag = 0;

% Communication / task allocation
% Options: 'stepwiseHungarian_unique' or 'none'
swarmModel.taskAllocation = 'stepwiseHungarian_unique';
swarmModel.samplesPerTask = 6;
swarmModel.bundleSize = 2;
swarmModel.knnNumber = 8;

% Task Generation
% Options 'mutualInfoWpts' , 'randomWpts' , 'lawnmower'
swarmModel.taskGeneration = 'mutualInfoWpts';
swarmModel.numTasks = 75;
swarmModel.stepSizeGain = 0.2;
swarmModel.percentTol = 0.05;
swarmModel.maxIters = 100;

% Mapping
swarmModel.nG = 25; % number of discrete sensor levels
swarmModel.mG = 3; % sensitivity
swarmModel.mapConfLevel = 0.95;
swarmModel.inc = 2;% kriging, deg
swarmModel.npeaks = 4;
swarmModel.ax = 1;
swarmModel.ay = 0.01;
    
% Detection / Tracking (LRDT)
swarmModel.LRDTOnTheFlyFlag = 1;
swarmModel.nodeDensityInitGuess = 1/6; % used on first step before kriging takes place
swarmModel.probAbsentPrior = 0.50; % for initialization
swarmModel.decayRate = 0.05; % value from 0 to 1
swarmModel.terminateSimOnDetect = 0;
swarmModel.confLevel = 0.95;
swarmModel.mZ = 3;
swarmModel.nZ = 25;

% Target
targetModel = struct;
% options: 'constantSpeedRandomWalk' or 'generative'
targetModel.type = 'constantSpeedRandomWalk';
targetModel.M = 1; % number of targets
targetModel.probStopping = 0.75;
targetModel.m = 1.0;
targetModel.d = 0.1;
targetModel.inertia = 100; % a value greater than zero

% Environment/Map
% load environment the full map
trueWorld = struct;
trueWorld.type = 'cityblocksAtF3'; % 'cityblocks', %'openStreetMap', 'osmAtF3'
trueWorld.f3Workspace = 'right-square'; % 'full', 'right-square'
trueWorld.binWidth = 0.75; % distance used to declare two nodes as connected (use 7 for open street map)
trueWorld.borderOffset = 2*trueWorld.binWidth; % used for adding padding to the map
trueWorld.folder = './data/'; % folder with map file
trueWorld.fileName = 'cityblocksAtF3';
trueWorld.blockLength = 6;
trueWorld.numBlocks = 3;


% derived
[runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = derivedParams(runParams, ROS_MACE, trueWorld, swarmModel, targetModel);



end
