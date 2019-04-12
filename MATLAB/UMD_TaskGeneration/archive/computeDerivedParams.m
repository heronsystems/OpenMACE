function [runParams, swarmModel, trueWorld, ROS_MACE] = computeDerivedParams(runParams, swarmModel, trueWorld, ROS_MACE, targetModel)

% agents follow a double integrator model with xdot = Ax + Bu and
% saturation on the input u. A, and B are defined below.
swarmModel.d = swarmModel.umax/swarmModel.vmax; % agent damping parameter
swarmModel.A = [0 0 1 0; 0 0 0 1; 0 0 -swarmModel.d 0; 0 0 0 -swarmModel.d];
swarmModel.B = [0 0; 0 0; 1 0; 0 1];

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

% for sensor model / msmt likelihood 
swarmModel.m = zTransform(swarmModel.Pd) - zTransform(swarmModel.Pfa);

% derived world model parameters
trueWorld = loadEnvironment(trueWorld, targetModel);

% given the sensing radius, measurements can only be obtained within some
% window of bins around the target
trueWorld.windowWidth = 3*ceil(swarmModel.Rsense/trueWorld.binWidth)+1;
trueWorld.windowWidth = 3*ceil(swarmModel.Rsense/trueWorld.binWidth)+1;
end