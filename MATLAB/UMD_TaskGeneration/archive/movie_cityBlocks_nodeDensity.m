function movie_cityBlocks_nodeDensity( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figure;
rows = 2;
cols = 2;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,2);
spArray(3) = subplot(rows,cols,[3 4]);
% options
%   'GroundTruth',
%   'LikelihoodTargetStateExplored',
%   'FrontierWpts',
%   'LikelihoodOccupancyGraphExplored',
%   'CellAge',
%   'CellState',
%   'KrigingInterp'
spTypes = {'LikelihoodOccupancyGraphExplored','PredictedNodes','EstNodeDensity'};
playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);
end