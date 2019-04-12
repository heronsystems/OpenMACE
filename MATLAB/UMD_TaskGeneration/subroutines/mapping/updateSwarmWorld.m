function swarmWorld = updateSwarmWorld(swarmWorld, swarmState, swarmModel, trueWorld, targetModel, targetState)

% figure;
% subplot(2,2,1)
% imagesc(swarmWorld.V); caxis([0 1])
% subplot(2,2,2)
% imagesc(swarmWorld.U); caxis([0 1])
% subplot(2,2,3)
% imagesc(swarmWorld.O); caxis([0 1])
% subplot(2,2,4)
% imagesc(swarmWorld.O./swarmWorld.U);
% colorbar;
% title('Initial')

% simulateGridCellSensor
switch swarmModel.mappingSensorType
    case 'perfect'
        [cellsInView, discoveredCells, nodeCells, swarmWorld.cellStateMat, swarmWorld.cellMsmtMat ] = simulateGridCellSensor( ...
            trueWorld.xcp, trueWorld.ycp, swarmModel.Rsense, swarmState.x, swarmModel.N, ...
            swarmWorld.cellStateMat, swarmWorld.cellMsmtMat, trueWorld.numNodesMat, swarmModel.communicationTopology );
        detectedCells = nodeCells;
    case 'noisy'
        % map and target sensor
        [cellsInView, mapSignals, targSignals, swarmWorld.cellStateMat, swarmWorld.cellMsmtMat ] = simulateNoisySensors( trueWorld.xcp, trueWorld.ycp, swarmModel.Rsense, swarmState.x, ...
            swarmModel.N, targetState, targetModel, swarmWorld.cellStateMat, swarmWorld.cellMsmtMat, trueWorld.numNodesMat, swarmModel.communicationTopology, ...
            swarmModel.mG, swarmModel.nG, swarmModel.mZ, swarmModel.nZ, trueWorld.G_env );

        % Bayes update at explored cells
        [swarmWorld.V, swarmWorld.U, swarmWorld.O] = bayesUpdate( swarmWorld.cellDetMat , swarmWorld.V, swarmWorld.U, swarmWorld.O, cellsInView, mapSignals, targSignals, ...
            swarmModel.g_V, swarmModel.g_UO, swarmModel.z_VU, swarmModel.z_O );
%         figure;
%         subplot(2,2,1)
%         imagesc(swarmWorld.V); caxis([0 1])
%         subplot(2,2,2)
%         imagesc(swarmWorld.U); caxis([0 1])
%         subplot(2,2,3)
%         imagesc(swarmWorld.O); caxis([0 1])
%         subplot(2,2,4)
%         imagesc(swarmWorld.O./swarmWorld.U);
%         colorbar;
%         title('bayes update')
        
        
        % check for new detections
        [swarmWorld.cellDetMat, detectedCells, swarmWorld.V, swarmWorld.U, swarmWorld.O] = detectNodes( swarmWorld.cellDetMat , swarmWorld.V, swarmWorld.U, swarmWorld.O, cellsInView, swarmModel.nodeLRthresh );
        
%         figure;
%         subplot(2,2,1)
%         imagesc(swarmWorld.V); caxis([0 1])
%         subplot(2,2,2)
%         imagesc(swarmWorld.U); caxis([0 1])
%         subplot(2,2,3)
%         imagesc(swarmWorld.O); caxis([0 1])
%         subplot(2,2,4)
%         imagesc(swarmWorld.O./swarmWorld.U);
%         colorbar;
%         title('detected nodes')
        
end




% predict new nodes
% [predictedNodeProbMat, nodeDensityEstimate] = predictNewNodes(swarmWorld.cellStateMat, trueWorld.xx, trueWorld.yy, swarmModel.maxUnexploredPrior, swarmModel.mapping.krigingSigma, swarmModel.nodeDensityInitGuess);

% figure;
% subplot(2,2,1)
% imagesc(swarmWorld.V); caxis([0 1])
% subplot(2,2,2)
% imagesc(swarmWorld.U); caxis([0 1])
% subplot(2,2,3)
% imagesc(swarmWorld.O); caxis([0 1])
% subplot(2,2,4)
% imagesc(swarmWorld.O./swarmWorld.U);
% colorbar;
% title('PREupdateUnexploredPror')

% update unexplored prior
[swarmWorld.V, swarmWorld.U, swarmWorld.O] = updateUnexploredPrior(swarmWorld.cellStateMat, trueWorld.xx, trueWorld.yy, swarmModel.mapping.krigingSigma, numnodes(swarmWorld.exploredGraph), swarmModel.probAbsentPrior, swarmWorld.V, swarmWorld.U, swarmWorld.O);

% figure;
% subplot(2,2,1)
% imagesc(swarmWorld.V); caxis([0 1]); colorbar;
% subplot(2,2,2)
% imagesc(swarmWorld.U); caxis([0 1]); colorbar;
% subplot(2,2,3)
% imagesc(swarmWorld.O); colorbar;
% subplot(2,2,4)
% imagesc(swarmWorld.O./swarmWorld.U);
% colorbar;
% title('updateUnexploredPror')


% figure;
% plot(swarmWorld.log_likelihood)
% title('updateUnexploredPror ')

% buildOccupancyGraphOnTheFly
[swarmWorld, numberPreviouslyExploredNodes] = buildOccupancyGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, detectedCells );


% figure;
% plot(swarmWorld.log_likelihood)
% title('updateUnexploredPror')

%     % simulateFrontierNodeSensor
%     swarmWorld.frontierIndex = simulateFrontierNodeSensor(swarmWorld.exploredGraph, ...
%     trueWorld.G_env, swarmWorld.frontierIndex, numberPreviouslyExploredNodes);

% compute metrics: percentage of explored nodes
swarmWorld.mapPercentage = [swarmWorld.mapPercentage; (numnodes(swarmWorld.exploredGraph)-1)/length(trueWorld.nodeX)];


end

