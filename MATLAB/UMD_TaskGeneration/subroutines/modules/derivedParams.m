function [runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = derivedParams(runParams, ROS_MACE, trueWorld, swarmModel, targetModel)
% agents follow a double integrator model with xdot = Ax + Bu and
% saturation on the input u. A, and B are defined below.
swarmModel.d = swarmModel.umax/swarmModel.vmax; % agent damping parameter
swarmModel.A = [0 0 1 0; 0 0 0 1; 0 0 -swarmModel.d 0; 0 0 0 -swarmModel.d];
swarmModel.B = [0 0; 0 0; 1 0; 0 1];
runParams.Tsamp = swarmModel.Tsamp; % make a copy

swarmModel.planningHorizon = swarmModel.samplesPerTask * swarmModel.Tsamp; %runParams.T; %

% mapping
swarmModel.nodeLRthresh = swarmModel.mapConfLevel / (1 - swarmModel.mapConfLevel); % initial value
swarmModel.gval = linspace(-3,swarmModel.mG+3,swarmModel.nG);
for l = 1:1:swarmModel.nG
    swarmModel.g_V(l) = exp(-(swarmModel.gval(l))^2/2);
    swarmModel.g_UO(l) = exp(-(swarmModel.gval(l)-swarmModel.mG)^2/2);
end
swarmModel.g_V = swarmModel.g_V ./ sum(swarmModel.g_V);
swarmModel.g_UO = swarmModel.g_UO ./ sum(swarmModel.g_UO);
swarmModel.numNodesEstPercent = swarmModel.nodeDensityInitGuess;

% tracking
swarmModel.q_s_n =  swarmModel.decayRate*(1-swarmModel.probAbsentPrior);
swarmModel.q_n_s =  swarmModel.decayRate*swarmModel.probAbsentPrior;
swarmModel.LReqbm = swarmModel.q_s_n / swarmModel.q_n_s; % note LR reaches eqbm value equal to q_s_n/q_n_s
swarmModel.q_n_n = 1 - swarmModel.q_s_n;
swarmModel.cumlLRthresh = swarmModel.confLevel / (1 - swarmModel.confLevel); % initial value
swarmModel.zval = linspace(-3,swarmModel.mZ+3,swarmModel.nZ);
for q = 1:1:swarmModel.nZ
    swarmModel.z_VU(q) = exp(-(swarmModel.zval(q))^2/2);
    swarmModel.z_O(q) = exp(-(swarmModel.zval(q)-swarmModel.mZ)^2/2);
end
swarmModel.z_VU = swarmModel.z_VU ./ sum(swarmModel.z_VU);
swarmModel.z_O = swarmModel.z_O ./ sum(swarmModel.z_O);

% target
targetModel.restPeriod = swarmModel.Tsamp;


%
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
