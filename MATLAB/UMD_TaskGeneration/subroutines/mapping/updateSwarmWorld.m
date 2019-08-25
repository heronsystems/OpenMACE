function swarmWorld = updateSwarmWorld(swarmWorld, swarmState, swarmModel, trueWorld, targetModel, targetState)

% map and target sensor
[swarmWorld.cellsInView, swarmWorld.mapSignals, swarmWorld.targSignals, swarmWorld.cellStateMat, swarmWorld.numViews ] = simulateNoisySensors( trueWorld.xcp, trueWorld.ycp, swarmModel.Rsense, swarmState.x, ...
    swarmModel.N, targetState, targetModel, swarmWorld.cellStateMat, trueWorld.bin2NodeID, ...
    swarmModel.mG, swarmModel.nG, swarmModel.mZ, swarmModel.nZ, trueWorld.G_env, swarmWorld.numViews );

% Bayes update at explored cells
[swarmWorld.V, swarmWorld.U, swarmWorld.O] = bayesUpdate( swarmWorld.cellDetMat , swarmWorld.V, swarmWorld.U, swarmWorld.O, swarmWorld.cellsInView, swarmWorld.mapSignals, swarmWorld.targSignals, ...
    swarmModel.g_V, swarmModel.g_UO, swarmModel.z_VU, swarmModel.z_O );

% check for new detections
[swarmWorld.cellDetMat, detectedCells, swarmWorld.V, swarmWorld.U, swarmWorld.O] = detectNodes( swarmWorld.cellDetMat , swarmWorld.V, swarmWorld.U, swarmWorld.O, swarmWorld.cellsInView, swarmModel.nodeLRthresh );

% buildOccupancyGraphOnTheFly
[swarmWorld] = buildOccupancyGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, detectedCells );

% update unexplored prior
tic;
[swarmWorld.V, swarmWorld.U, swarmWorld.O] = updateUnexploredPrior(swarmWorld.cellStateMat, trueWorld.xx, trueWorld.yy, numnodes(swarmWorld.exploredGraph), swarmModel.probAbsentPrior, swarmWorld.V, swarmWorld.U, swarmWorld.O, swarmWorld.edgeDir, swarmModel.inc, swarmModel.npeaks, swarmModel.ax, swarmModel.ay);
disp('updateUnexploredPrior')
toc;

% compute metrics: percentage of explored nodes
swarmWorld.mapPercentage = [swarmWorld.mapPercentage; (numnodes(swarmWorld.exploredGraph)-1)/length(trueWorld.nodeX)];


end

