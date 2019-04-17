function [swarmWorldHist, swarmStateHist, targetStateHist] = simulateMACE(targetState, targetModel, swarmState, swarmModel, swarmWorld, trueWorld, runParams, ROS_MACE)

s = 1;
for i = 1:1:swarmModel.N
    swarmEast(s,i) = swarmState.x(4*i-3,1);
    swarmNorth(s,i) = swarmState.x(4*i-2,1);
end

% prepare run
tStart = tic;
tSample = swarmModel.Tsamp;
t(1) = 0;
tNow = 0;
k = 2;

targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams);
while ( tNow <= runParams.T )
    % we continously collect the latest data until the sample time is
    % reached
    tSampleStart = tic;
    agentUpdated = zeros(1,ROS_MACE.N);
    msgCollection = cell(1,ROS_MACE.N);
    
    % the following while-loop guarantees as many as agents get an update
    % within the maximum allowed time
    while ( toc(tSampleStart) <= tSample/4 ) && (~all(agentUpdated)) 
        % the factor for tSample depends on the firing rate of 
        % /MACE/UPDATE_POSITION topic
        msg = ROS_MACE.positionSub.LatestMessage;
        
        % store the msg from the corresponding agent
        agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
        msgCollection{agentIndex} = msg;
        agentUpdated(agentIndex) = 1;
        
        switch ROS_MACE.coordSys
            case 'ENU'
                swarmState.x(4*agentIndex-3,1) = msg.Easting;
                swarmState.x(4*agentIndex-2,1) = msg.Northing;
                swarmState.x(4*agentIndex-1,1) = -1; % unused for now
                swarmState.x(4*agentIndex,1) = -1;
            case 'F3'
                [xF3, yF3] = ENUtoF3(msg.Easting, msg.Northing);
                swarmState.x(4*agentIndex-3,1) = xF3;
                swarmState.x(4*agentIndex-2,1) = yF3;
                swarmState.x(4*agentIndex-1,1) = -1; % unused for now
                swarmState.x(4*agentIndex,1) = -1;
        end
    end
    
    tNow = toc(tStart);
    dt = tNow - tSample + swarmModel.Tsamp;
    tSample = tSample + swarmModel.Tsamp;
    swarmState.t = tNow;
    fprintf('Measurement No. %d (Time = %3.3f sec) \n', swarmState.k, swarmState.t);
    
    for k = 1:ROS_MACE.N
        positionCallback( ROS_MACE, msgCollection{k} );
    end
    
    % simulate measurements/update the swarmWorld
    swarmWorld = updateSwarmWorld(swarmWorld, swarmState, swarmModel, trueWorld, targetModel, targetState);
    
    % update likelihood surface
    swarmWorld = updateLikelihood(swarmWorld, swarmState, swarmModel, trueWorld, targetState, targetModel);
    [swarmWorld.entropyMat, swarmWorld.mutualInfoSurface, swarmWorld.totalEntropy] = mutualInformationMappingTarget(swarmWorld.V, swarmWorld.U, swarmWorld.O, swarmModel.z_VU, swarmModel.z_O, swarmModel.g_V, swarmModel.g_UO);
    
    % simulate targets
    integerTime = 1+swarmState.k/floor(swarmModel.Tsamp/runParams.dt);
    targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams, integerTime);
    
    
    
    % update tasks only after several samples
    if ( mod( swarmState.k , swarmModel.samplesPerTask ) == 0 )
        fprintf('*** New tasks generated. *** \n');
        % generate/allocate tasks at new sampling times
        % task generation
        [tasks, swarmWorld] = taskGeneration(swarmWorld, swarmModel, trueWorld);
        % task allocation
        [swarmState, swarmWorld] = taskAllocation(tasks, swarmState, swarmModel, swarmWorld, trueWorld, runParams);
        % TODO: check for collision, modify wpts accordingly
    end
    
    
    [swarmState] = taskManagement(swarmState, swarmModel, swarmWorld);
    
    % dispatch waypoint comamands
    updateWpts( ROS_MACE, [swarmState.xd' swarmState.yd'] );
    
    % plot the task bundle using variables ==========

    subplot(ROS_MACE.taskAndLocation);
    colors=['rbkmgcy'];
    
    
    if isempty(ROS_MACE.tempHandle{1,1})
        for k = 1:ROS_MACE.N
            ROS_MACE.tempHandle{1,k} = plot(swarmWorld.cellCenterOfMass(swarmState.wptList(k,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(k,:),2),[colors(k) '-']);
            ROS_MACE.tempHandle{2,k} = plot(swarmWorld.cellCenterOfMass(swarmState.wptList(k,swarmState.wptIndex(k)),1),swarmWorld.cellCenterOfMass(swarmState.wptList(k,swarmState.wptIndex(k)),2),[colors(k) '+'],'MarkerSize',4,'linewidth',2);
        end
    else
        for k = 1:ROS_MACE.N
            set(ROS_MACE.tempHandle{1,k},'XData', swarmWorld.cellCenterOfMass(swarmState.wptList(k,:),1),'YData',swarmWorld.cellCenterOfMass(swarmState.wptList(k,:),2));
            set(ROS_MACE.tempHandle{2,k},'Xdata', swarmWorld.cellCenterOfMass(swarmState.wptList(k,swarmState.wptIndex(k)),1),'YData',swarmWorld.cellCenterOfMass(swarmState.wptList(k,swarmState.wptIndex(k)),2));
        end
    end
    drawnow;
    
    % ========================
    
    
    % save the swarm world for later use
    swarmWorldHist{s} = swarmWorld;
    swarmStateHist{s} = swarmState;
    targetStateHist{s} = targetState;
    s = s + 1;
    swarmState.k = s;
    targetState.k = s;
    
end
land( ROS_MACE );
% TODO: Check batteries and land if low
% TODO: Put main while loop in a try-catch, handle errors by landing quads


end