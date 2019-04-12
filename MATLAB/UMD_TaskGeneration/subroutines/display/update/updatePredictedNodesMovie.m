function plotHandles = updatePredictedNodesMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
set(plotHandles.figh_subplot2,'CData',swarmWorld.predictedNodeProbMat);
end