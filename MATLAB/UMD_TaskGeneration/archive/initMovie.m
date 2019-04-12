function plotHandles = initMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel, sp_handle, sp_type)
% initialize states
k = 1;
swarmState = swarmStateHist{k};
targetState = targetStateHist{k};
swarmWorld = swarmWorldHist{k};

switch sp_type
    case 'groundTruth'
        plotHandles  = initGroundTruthMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, sp_handle);
    case 'likelihoodOccupancyGraphExplored'
        plotHandles = initLikelihoodOccupancyGraphExploredMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, sp_handle);
    case 'likelihoodTargetStateExplored' %
        plotHandles = initLikelihoodTargetStateExploredMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, sp_handle);
    case 'likelihoodTargetStateFull'
        plotHandles = initLikelihoodTargetStateFullMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, sp_handle);
    case 'frontierWpts'
        plotHandles = initFrontierWptsMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, sp_handle);
    case 'frontierWpts2'
        plotHandles = initFrontierWptsMovie2(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, sp_handle);
    case 'frontierWpts3'
        plotHandles = initFrontierWptsMovie3(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, sp_handle);
    case 'cellState'
        plotHandles = initCellStateMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, sp_handle);
end



end
