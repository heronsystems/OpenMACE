function [runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams()

% simulation
runParams = struct;
runParams.type = 'matlab'; % 'matlab' 'mace' 'f3'
runParams.T = 30; % total simulation/mission time
runParams.dt = 0.1; % time-step (even if MACE is running, Sheng needs this for cost computation)
runParams.flags.movie = 1; % play movie
runParams.movie.option = 'likelihood'; % 'samplingPriority' or 'scalarSurfaces' or 'likelihood'
runParams.movie.useBackgroundImg = 1; %
if ( runParams.movie.useBackgroundImg )
    %     % Randall's Island
    %     runParams.movie.backgroundImgFile = './data/RNDIL_bright.png';
    %     runParams.movie.backgroundImgSWCornerLat = 40.786700000000003;
    %     runParams.movie.backgroundImgSWCornerLong = -73.926100000000005;
    %     runParams.movie.backgroundImgNECornerLat = 40.796199999999999;
    %     runParams.movie.backgroundImgNECornerLong = -73.911799999999999;
    % F3
    runParams.movie.backgroundImgFile = './data/f3_bright.png';
    runParams.movie.backgroundImgBottomLeftCornerX = -73;
    runParams.movie.backgroundImgBottomLeftCornerY = -30;
    runParams.movie.backgroundImgWidth = 108;
    runParams.movie.backgroundImgHeight = 50;
end
ROS_MACE = [];
if ( strcmp(runParams.type, 'mace') )
    ROS_MACE = struct;
    %ROS_MACE.ip ='10.104.193.156'; % Artur Office
    ROS_MACE.ip ='192.168.1.219'; % Artur Home
    ROS_MACE.operationalAlt = 10; % m
    ROS_MACE.xInit = 0; % position of first quad in local frame
    ROS_MACE.yInit = 0;
    ROS_MACE.initSpacing = 3;
    ROS_MACE.coordSys = 'F3'; % 'ENU' or 'F3'
    % Heron Farm
    %ROS_MACE.LatRef = 37.889246;
    %ROS_MACE.LongRef = -76.814084;
    % F3
    ROS_MACE.LatRef = 38.973699;
    ROS_MACE.LongRef = -76.921897;
    % Randall's Island NY
    %ROS_MACE.LatRef = 40.791450;
    %ROS_MACE.LongRef = -73.918950;
    ROS_MACE.startOnPerimeter = 1; % 0/1 flag
    ROS_MACE.trails = 10; % length of visual trail, in no. samples
end

% load environment
trueWorld = struct;
trueWorld.type = 'cityblocks'; % 'goxel', 'cityblocks', %'openStreetMap'
trueWorld.borderOffset = 0; % used for adding padding to the map
trueWorld.binWidth = 1; % distance used to declare two nodes as connected (use 7 for open street map)
trueWorld.folder = './data/'; % folder with map file
trueWorld.binWidth=1;
% manipulate the map by shifting and truncating
trueWorld.mapManipFlag = 1;
if ( trueWorld.mapManipFlag )
    % first the nodes are clipped to be inside the interval:
    trueWorld.clipXmin = 0;
    trueWorld.clipXmax = 108;
    trueWorld.clipYmin = 0;    
    trueWorld.clipYmax = 50;    
    % then the map is shifted 
    trueWorld.shiftX = -73;
    trueWorld.shiftY = -30;
end
switch trueWorld.type
    case 'goxel'
        trueWorld.fileName = 'martin_small_flat.txt'; % map file from goxel
    case 'cityblocks'
        trueWorld.fileName = 'cityblocks';
        trueWorld.blockLength = 10;
        trueWorld.numBlocks = 4;
    case 'openStreetMap'
        trueWorld.fileName = 'RandallsIsland_Big.osm';
        trueWorld.binWidth = trueWorld.binWidth;
        trueWorld.nodeFactor = 0.95;
        trueWorld.edgeFactor = 1.75;
        trueWorld.dim = 400;
end

% search agent properties
swarmModel = struct;
swarmModel.N = 4; % number of agents
swarmModel.Rsense = 3; % sensing radius
swarmModel.Tsamp = 1; % sample time
swarmModel.samplesPerTask = 8;
swarmModel.taskGeneration = 'frontierWpts'; % 'randomWpts', 'voronoiWpts', or 'frontierWpts'
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
        swarmModel.mapping.method = 'frontierOnly'; % options are: 'frontierOnly' or 'frontierAndBlob'
        swarmModel.mapping.minBlobArea = pi*swarmModel.Rsense^2*2;  % threshold for the minimum area of a blob
        swarmModel.mapping.maxMajorMinorAxisRatio = 10; % threshold for the ratio between majoraxislength and minor axis length of a subblob, give inf if you do not want to apply this filter
        swarmModel.mapping.blobCostScale = 1.4; % scale the cost for reaching a subblob centroid by this amount
    case 'voronoiWpts'
        swarmModel.numTasks = 50;
        swarmModel.stepSizeGain = 0.2;
        swarmModel.percentTol = 0.03;
        swarmModel.maxIters = 500;
        swarmModel.explorationPriority = 0.2; % AW: errors when this value approaches 1
end
swarmModel.mapping.krigingSigma = 1; % controls how much kriging interp diffuses
swarmModel.mapping.krigingMethod = 'recursive'; % options are: 'recursive' or 'standard'

% target properties
targetModel = struct;
targetModel.M = 10; % number of targets
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
swarmModel.Pd =0.95;
swarmModel.Pfa = 0.10;
swarmModel.positionSensorStDev = 1;
swarmModel.decayFactor = 0.001;
% for initialization
swarmModel.probAbsent = 0.75;

% computed derived parameters, using the above inputs
[runParams, swarmModel, trueWorld, ROS_MACE] = computeDerivedParams(runParams, swarmModel, trueWorld, ROS_MACE, targetModel);

end
