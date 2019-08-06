function [swarmWorldHist, swarmStateHist, targetStateHist] = simulateMatlab(targetState, targetModel, swarmState, swarmModel, swarmWorld, trueWorld, runParams)

% histories are used for performance analysis and plotting
swarmWorldHist{1} = swarmWorld;
swarmStateHist{1} = swarmState;
targetStateHist{1} = targetState;

% init
s = 1; % number of samples
t(1) = 0;        

% simulate targets
targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams, 1);

for k = 2:1:runParams.Nsim
    disp('---- Begin Loop ----');
    tLoopStart = tic;
    % run only at sample times
    sampleTimeFlag = mod( swarmState.k , floor(swarmModel.Tsamp/runParams.dt) ) == 0;
    if ( sampleTimeFlag )
        disp('**** Processing Measurements and Tasking ****');
        % current time
        swarmWorld.time = k*runParams.dt;
        
        % update swarm world
        tTemp = tic;
        swarmWorld = updateSwarmWorld(swarmWorld, swarmState, swarmModel, trueWorld, targetModel, targetState);
        disp('updateSwarmWorld');
        toc(tTemp);
        
        % update likelihood
        tTemp = tic;        
        swarmWorld = updateLikelihood(swarmWorld, swarmState, swarmModel, trueWorld, targetState, targetModel);
        disp('updateLikelihood');
        toc(tTemp);
        
        % mutual info
        tTemp = tic;
        [swarmWorld.entropyMat, swarmWorld.mutualInfoSurface, swarmWorld.totalEntropy] = mutualInformationMappingTarget(swarmWorld.V, swarmWorld.U, swarmWorld.O, swarmModel.z_VU, swarmModel.z_O, swarmModel.g_V, swarmModel.g_UO);
        disp('mutualInfo');
        toc(tTemp);
        
        % simulate targets
        tTemp = tic;
        integerTime = 1+swarmState.k/floor(swarmModel.Tsamp/runParams.dt);        
        targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams, integerTime);
        disp('targetMotionUpdate');
        toc(tTemp);
        
        % task generation 
        tTemp = tic;
        [tasks, swarmWorld] = taskGeneration(swarmWorld, swarmModel, trueWorld);
        disp('taskGeneration');
        toc(tTemp);        
        
        % task allocation
        tTemp = tic;
        [swarmState, swarmWorld] = taskAllocation(tasks, swarmState, swarmModel, swarmWorld, trueWorld, runParams);
        disp('taskAllocation');
        toc(tTemp);        
        
    end
    
    % runs at every time-step
    tTemp = tic;
    [swarmState] = taskManagement(swarmState, swarmModel, swarmWorld);
    disp('taskManagement');
    toc(tTemp); 
        
    % save data at each sampling time
    if ( sampleTimeFlag )
        swarmWorldHist{s} = swarmWorld;
        swarmStateHist{s} = swarmState;
        targetStateHist{s} = targetState;
        s = s + 1;
    end
    
    % simulate searchers
    xdot = swarmDynamics(swarmState, swarmModel);
    swarmState.x = swarmState.x + xdot' * runParams.dt; % euler's method
    t(k) = t(k-1) + runParams.dt;
    
    % display
    if ( mod(k,floor(runParams.Nsim/50)) == 0 || k == 2 )
        fprintf('**** Run Percent : %3.0f **** \n', k/runParams.Nsim*100)
    end
    
    swarmState.t = t(k);
    swarmState.k = k;
    targetState.k = k;
    
    
    disp('Loop Took:');
    toc(tLoopStart);
end
end