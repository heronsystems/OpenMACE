function swarmWorld = updateSwarmWorld(swarmWorld, swarmState, swarmModel, trueWorld, targetModel, targetState)

% map and target sensor
[cellsInView, mapSignals, targSignals, swarmWorld.cellStateMat, swarmWorld.cellMsmtMat ] = simulateNoisySensors( trueWorld.xcp, trueWorld.ycp, swarmModel.Rsense, swarmState.x, ...
    swarmModel.N, targetState, targetModel, swarmWorld.cellStateMat, swarmWorld.cellMsmtMat, trueWorld.numNodesMat, swarmModel.communicationTopology, ...
    swarmModel.mG, swarmModel.nG, swarmModel.mZ, swarmModel.nZ, trueWorld.G_env );

% Bayes update at explored cells
[swarmWorld.V, swarmWorld.U, swarmWorld.O] = bayesUpdate( swarmWorld.cellDetMat , swarmWorld.V, swarmWorld.U, swarmWorld.O, cellsInView, mapSignals, targSignals, ...
    swarmModel.g_V, swarmModel.g_UO, swarmModel.z_VU, swarmModel.z_O );

% check for new detections
[swarmWorld.cellDetMat, detectedCells, swarmWorld.V, swarmWorld.U, swarmWorld.O] = detectNodes( swarmWorld.cellDetMat , swarmWorld.V, swarmWorld.U, swarmWorld.O, cellsInView, swarmModel.nodeLRthresh );

% buildOccupancyGraphOnTheFly
[swarmWorld] = buildOccupancyGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, detectedCells );

% update unexplored prior
[swarmWorld.V, swarmWorld.U, swarmWorld.O] = updateUnexploredPrior(swarmWorld.cellStateMat, trueWorld.xx, trueWorld.yy, numnodes(swarmWorld.exploredGraph), swarmModel.probAbsentPrior, swarmWorld.V, swarmWorld.U, swarmWorld.O, swarmWorld.edgeDir);

% compute metrics: percentage of explored nodes
swarmWorld.mapPercentage = [swarmWorld.mapPercentage; (numnodes(swarmWorld.exploredGraph)-1)/length(trueWorld.nodeX)];


end

