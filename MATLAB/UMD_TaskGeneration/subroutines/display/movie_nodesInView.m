function movie_nodesInView( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figure;
rows = 2;
cols = 2;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,2);
spArray(3) = subplot(rows,cols,3);
spArray(4) = subplot(rows,cols,4);
% spArray(5) = subplot(rows,cols,5);
% spArray(6) = subplot(rows,cols,6);
% options
%   'GroundTruth',
%   'LikelihoodTargetStateExplored',
%   'FrontierWpts',
%   'LikelihoodOccupancyGraphExplored',
%   'CellAge',
%   'CellState',
%   'KrigingInterp'
spTypes = {'LikelihoodOccupancyGraphExplored','NodesInView','ProbPresence','CumlLR'};
playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);
end