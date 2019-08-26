function [swarmWorldHist, swarmStateHist, targetStateHist] = simulateMACE(targetState, targetModel, swarmState, swarmModel, swarmWorld, trueWorld, runParams, ROS_MACE)

if exist('bundleSource','var')
    clear global bundleSource
end

global bundleSource
if strcmp(ROS_MACE.wptCoordinator,'standalone')
    if isempty(bundleSource)
        % the global bundleSource stores the current bundles.
        % each of the first N rows represents the bundle of an agent
        % the first column is the current bundle number, starting from 1
        % the second column is the waypoint altitude
        % the third to the twelfth columns represent easting1, northing1, easting2, northing2,..., easting5, northing5
        bundleSource = zeros(swarmModel.N,1+1+2*6);
        bundleSource(:,1) = 0;
        % the last row is used for status of the bundle
        % specifically, the first column of the last row has two values: 0 or 1
        % 0 means the bundleSource is being written and the value is not
        % complete (temporarily unreliable)
        % 1 means the bundleSource is ready to be read
        bundleSource = [bundleSource; zeros(1,1+1+2*6)];
    end
end

s = 1;
for i = 1:1:swarmModel.N
    swarmEast(s,i) = swarmState.x(4*i-3,1);
    swarmNorth(s,i) = swarmState.x(4*i-2,1);
end

% prepare run
tStart = tic;
tSampleStart = tic;
tSample = swarmModel.Tsamp;
t(1) = 0;
tNow = 0;
k = 2;

targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams);
while ( tNow <= runParams.T )
    tStartWhile = tic;
    % we continously collect the latest data until the sample time is
    % reached
    
    agentUpdated = zeros(1,ROS_MACE.N);
    msgCollection = cell(1,ROS_MACE.N);
    
    % the following while-loop guarantees as many as agents get an update
    % within the maximum allowed time
    while (~all(agentUpdated)) %|| ( toc(tSampleStart) <= tSample )
        % when not all agents are updated OR tSample has not been reached, stay in the while loop
        % but if tSample has been passed, then grab one immediate position update and jump out of the while loop
        
        msg = ROS_MACE.positionSub.LatestMessage;
        
        % store the msg from the corresponding agent
        agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
        msgCollection{agentIndex} = msg;
        agentUpdated(agentIndex) = 1;
        
        switch ROS_MACE.coordSys
            case 'ENU'
                swarmState.x(4*agentIndex-3,1) = msg.Easting;
                swarmState.x(4*agentIndex-2,1) = msg.Northing;
                swarmState.x(4*agentIndex-1,1) = 0; % unused for now
                swarmState.x(4*agentIndex,1) = 0;
            case 'F3'
                [xF3, yF3] = ENUtoF3(msg.Easting, msg.Northing);
                swarmState.x(4*agentIndex-3,1) = xF3;
                swarmState.x(4*agentIndex-2,1) = yF3;
                swarmState.x(4*agentIndex-1,1) = 0; % unused for now
                swarmState.x(4*agentIndex,1) = 0;
        end
        
        %         if ( toc(tSampleStart) > tSample )
        %             break;
        %         end
        
        if ( isfield(swarmState,'wptList') )
            [swarmState] = taskManagement(swarmState, swarmModel, swarmWorld);
            if strcmp(ROS_MACE.wptCoordinator,'integrated')  % integrated wptCoordinator as the original implementation
                updateWpts( ROS_MACE, [swarmState.xd' swarmState.yd'], swarmState.wptIndex );
            end
        end
        
    end
  
    % Approximate speed of agents by finite difference 
    % (needed for task allocation)
    for i = 1:1:swarmModel.N
        if ( s > 1 ) % (otherwise if s == 1 use default zero velocity)
        lastSwarmState = swarmStateHist{s-1};
        delX = swarmState.x(4*i-3,1) - lastSwarmState.x(4*i-3,1);
        delY = swarmState.x(4*i-2,1) - lastSwarmState.x(4*i-2,1);
        swarmState.x(4*i-1,1) = delX/runParams.T;
        swarmState.x(4*i,1) = delY/runParams.T;
        end
    end
    
    
    %
    tSampleStart = tic;
    
    time_stamp = toc(tStartWhile);
    fprintf('Position loop took %3.3f sec \n',time_stamp);
    
    tNow = toc(tStart);
    dt = tNow - tSample + swarmModel.Tsamp;
    swarmState.t = tNow;
    fprintf('Measurement No. %d (Time = %3.3f sec) \n', swarmState.k, swarmState.t);
    
%     for k = 1:ROS_MACE.N
%         positionCallback( ROS_MACE, msgCollection{k} );
%     end
%     
%     fprintf('Plot loop took %3.3f sec \n',toc(tStartWhile)-time_stamp);
%     time_stamp = toc(tStartWhile);
    
    % simulate measurements/update the swarmWorld
    swarmWorld = updateSwarmWorld(swarmWorld, swarmState, swarmModel, trueWorld, targetModel, targetState);
    
    fprintf('updateSwarmWorld took %3.3f sec \n', toc(tStartWhile)-time_stamp);
    time_stamp = toc(tStartWhile);
    
    % update likelihood surface
    swarmWorld = updateLikelihood(swarmWorld, swarmState, swarmModel, trueWorld, targetState, targetModel);
    [swarmWorld.entropyMat, swarmWorld.mutualInfoSurface, swarmWorld.totalEntropy] = mutualInformationMappingTarget(swarmWorld.V, swarmWorld.U, swarmWorld.O, swarmModel.z_VU, swarmModel.z_O, swarmModel.g_V, swarmModel.g_UO);
    
    fprintf('updateLikelihood took %3.3f sec \n', toc(tStartWhile)-time_stamp);
    time_stamp = toc(tStartWhile);
    
    % simulate targets
    integerTime = 1+swarmState.k/floor(swarmModel.Tsamp/runParams.dt);
    targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams, integerTime);
    
    fprintf('targetMotionUpdate took %3.3f sec \n', toc(tStartWhile)-time_stamp);
    time_stamp = toc(tStartWhile);
    
    % update tasks only after several samples
    if ( mod( swarmState.k , swarmModel.samplesPerTask ) == 0 )
        fprintf('*** New tasks generated. *** \n');
        % generate/allocate tasks at new sampling times
        % task generation
        [tasks, swarmWorld] = taskGeneration(swarmWorld, swarmModel, trueWorld);
        fprintf('taskGeneration took %3.3f \n',toc(tStartWhile)-time_stamp);
        time_stamp = toc(tStartWhile);
        
        % task allocation
        [swarmState, swarmWorld] = taskAllocation(tasks, swarmState, swarmModel, swarmWorld, trueWorld, runParams);
        fprintf('taskAllocation took %3.3f \n',toc(tStartWhile)-time_stamp);
        time_stamp = toc(tStartWhile);
        
        if strcmp(ROS_MACE.wptCoordinator,'standalone')
            % mark the bundle status as 'written in process'
            bundleSource(end,1) = 0;
            % insert global variable to record the bundles
            bundleSource(1:end-1,1) = bundleSource(1:end-1,1) + 1;
            bundleSource(1:end-1,2) = ROS_MACE.operationalAlt';
            for k = 1:swarmModel.N
                for j = 1:swarmModel.bundleSize
                    bundleSource(k,j*2+1:j*2+2) = swarmWorld.cellCenterOfMass(swarmState.wptList(k,j),:);
                end
            end
            % mark the bundle status as 'written complete'
            bundleSource(end,1) = 1;
            swarmState.bundleSource = bundleSource;
        end
        
    end
    
    
    [swarmState] = taskManagement(swarmState, swarmModel, swarmWorld);
    
    if strcmp(ROS_MACE.wptCoordinator,'integrated')
        % dispatch waypoint comamands
        updateWpts( ROS_MACE, [swarmState.xd' swarmState.yd'], swarmState.wptIndex );
        
        fprintf('taskManagement/updateWpts took %3.3f sec \n',toc(tStartWhile)-time_stamp);
        time_stamp = toc(tStartWhile);
        
    end
    % plot the task bundle using variables
    % ROS_MACE = plotTaskBundleRealTime(swarmWorld, swarmState, ROS_MACE);
    
    % save the swarm world for later use
    swarmWorldHist{s} = swarmWorld;
    swarmStateHist{s} = swarmState;
    targetStateHist{s} = targetState;
    s = s + 1;
    swarmState.k = s;
    targetState.k = s;
    
    while ( toc( tStartWhile ) <= tSample )
    end
    
    fprintf('While Loop Took %3.3f ---------------------- \n',toc(tStartWhile));
    
end
land( ROS_MACE );
if strcmp(ROS_MACE.wptCoordinator,'standalone')
    clear global bundleSource;
end
% TODO: Check batteries and land if low
% TODO: Put main while loop in a try-catch, handle errors by landing quads


end