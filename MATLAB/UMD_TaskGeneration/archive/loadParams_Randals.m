function [runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_Randals()

% simulation
runParams = struct;
runParams.type = 'matlab'; % 'matlab' 'mace' 'f3'
runParams.T = 10*60; % total simulation/mission time
runParams.dt = 0.1; % time-step (even if MACE is running, Sheng needs this for cost computation)
runParams.flags.movie = 1; % play movie

runParams.movie.plotF3Obstacles = 0;
if ( runParams.movie.plotF3Obstacles )
    runParams.movie.perimX = [-59 28 28 -59 -59];
    runParams.movie.perimY = [13 13 -13 -13 13];
    R = 2;
    numPts = 20;
    [runParams.movie.pole1x, runParams.movie.pole1y] = generateCircle(0, 0, R, numPts);
    [runParams.movie.pole2x, runParams.movie.pole2y] = generateCircle(-30, 0, R, numPts);
end
ROS_MACE = [];
if ( strcmp(runParams.type, 'mace') )
    ROS_MACE = struct;
    ROS_MACE.ip ='10.104.193.156'; % Artur Office
    %ROS_MACE.ip ='192.168.1.219'; % Artur Home
    ROS_MACE.operationalAlt = 10; % m
    ROS_MACE.xInit = 0; % position of first quad in local frame
    ROS_MACE.yInit = 0;
    ROS_MACE.initSpacing = 3;
    ROS_MACE.coordSys = 'F3'; % 'ENU' or 'F3'
    % F3
    ROS_MACE.LatRef = 38.973699;
    ROS_MACE.LongRef = -76.921897;
    ROS_MACE.startOnPerimeter = 1; % 0/1 flag
    ROS_MACE.trails = 10; % length of visual trail, in no. samples
end
% load environment
trueWorld = struct;
trueWorld.type = 'openStreetMap'; % 'cityblocks', %'openStreetMap', 'osmAtF3'
trueWorld.borderOffset = 0; % used for adding padding to the map
%trueWorld.binWidth = 1; % distance used to declare two nodes as connected (use 7 for open street map)
trueWorld.folder = './data/'; % folder with map file
trueWorld.fileName = 'RandallsIsland_Big.osm';
trueWorld.binWidth = 5;
trueWorld.refX = -300;
trueWorld.refY = -200;
trueWorld.angle = 0*pi/180;
trueWorld.boxlength = 400;
trueWorld.boxwidth = 400;
trueWorld.buffer = 0;

runParams.movie.option = 'likelihood_explored'; % 'samplingPriority' or 'scalarSurfaces' or 'likelihood', 'likelihood_explored'
runParams.movie.useBackgroundImg = 1; %
if ( runParams.movie.useBackgroundImg )
    runParams.movie.backgroundImgFile = './data/RNDIL_bright.png';
    runParams.movie.backgroundImgWidth = 1.193465669123339e+03; % calculated in test_imageCoords.m
    runParams.movie.backgroundImgHeight = 1.069453056855127e+03;
    [xne,yne,~] = deg2utm( 40.796200, -73.911800); % NE Corner of Randalls Island .PNG file;
    [xsw,ysw,~] = deg2utm( 40.786700,-73.926100); % SW Corner of Randalls Island .PNG file;
    [xorigin,yorigin,~] = deg2utm( 40.791449999999998, -73.918949999999995); 
    runParams.movie.backgroundImgBottomLeftCornerX = xsw - xorigin - trueWorld.refX;
    runParams.movie.backgroundImgBottomLeftCornerY = ysw - yorigin - trueWorld.refY;
    runParams.movie.plotBuffer = 5;    
end

% search agent properties
swarmModel = struct;
swarmModel.N = 4; % number of agents
swarmModel.Rsense = 15; % sensing radius
swarmModel.Tsamp = 1; % sample time

swarmModel.samplesPerTask = 10; % originally 10
swarmModel.taskGeneration = 'stepwiseHungarian'; % 'mutualInfoWpts','stepwiseHungarian','randomWpts', 'voronoiWpts', 'lawnmower', or 'frontierWpts'

if strcmp(swarmModel.taskGeneration,'stepwiseHungarian')
    swarmModel.bundleSize = 5;
end

swarmModel.Told = 60; % sec, time used for saturated pixel "age".
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
        swarmModel.percentTol = 0.0025;
        swarmModel.maxIters = 100;
        %swarmModel.wptChangePeriod = 10; % sec, how often we generate a new random wpt
    case 'stepwiseHungarian'
        swarmModel.numTasks = 100;
        swarmModel.stepSizeGain = 0.2;
        swarmModel.percentTol = 0.03;
        swarmModel.maxIters = 500;
end
swarmModel.mapping.krigingSigma = 10; % controls how much kriging interp diffuses
swarmModel.mapping.krigingMethod = 'recursive'; % options are: 'recursive' or 'standard'
swarmModel.explorationPriority = 0.2; % for sampling priority surface

swarmModel.communicationTopology = 'centralized';   % options are: 'centralized' or 'allToAll'
swarmModel.taskAllocation = 'Hungarian'; % options are: 'Hungarian' or 'Auctioneer';
swarmModel.utilityComputation = 'computeInformationGain'; %LinearPath'; % options are: 'computeEnergyAndPenalty' or 'computeInformationGain'
swarmModel.planningHorizon = swarmModel.samplesPerTask * swarmModel.Tsamp;

% target properties
targetModel = struct;
targetModel.M = 2; % number of targets
targetModel.type = 'constantSpeedRandomWalk'; % 'varyingSpeedRandomWalk' or 'constantSpeedRandomWalk'
targetModel.probStopping = 0.40;
targetModel.m = 1.0;
targetModel.d = 0.1;
switch targetModel.type
    case 'varyingSpeedRandomWalk'
        targetModel.maxSpeed = 10;
    case 'constantSpeedRandomWalk'
        targetModel.restPeriod = swarmModel.Tsamp*1;
        targetModel.inertia = 100; % a value greater than zero
end

% tracking
swarmModel.LRDTOnTheFlyFlag = 1;
swarmModel.Pd =0.95;
swarmModel.Pfa = 0.10;
swarmModel.positionSensorStDev = 1;
swarmModel.decayFactor = 0.1;

% voronoi mutual information
swarmModel.mutualInfoSurfaceBlurFlag = 0;
swarmModel.maxUnexploredPrior = 0.85;
swarmModel.sensorDiscretizationLevels = 100;


% for initialization
swarmModel.probAbsent = 0.75;

% computed derived parameters, using the above inputs
[runParams, swarmModel, trueWorld, ROS_MACE] = computeDerivedParams(runParams, swarmModel, trueWorld, ROS_MACE, targetModel);

end
