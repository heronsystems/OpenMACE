function [swarmWorld, swarmState, targetState, ROS_MACE] = initializeRun(trueWorld, swarmModel, targetModel, runParams, ROS_MACE)

% initializeSwarmWorld
swarmWorld = initializeSwarmWorld(trueWorld , swarmModel, runParams);

% initializeTargetState
targetState = initializeTargetState(trueWorld, targetModel);

% initialize swarm state
[swarmState, ROS_MACE] = initializeSwarmState(swarmModel, trueWorld, runParams, ROS_MACE);

% initialize auctioneer
if strcmp(swarmModel.taskAllocation,'Auctioneer')
    [swarmWorld] = initializeAuctioneer(swarmWorld);    
end

end