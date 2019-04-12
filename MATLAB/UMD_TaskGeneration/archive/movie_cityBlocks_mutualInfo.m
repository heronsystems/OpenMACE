function movie_cityBlocks_mutualInfo( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figure;
rows = 3;
cols = 2;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,3);
spArray(3) = subplot(rows,cols,5);
spArray(4) = subplot(rows,cols,2);
spArray(5) = subplot(rows,cols,4);
spArray(6) = subplot(rows,cols,6);
% spArray(6) = subplot(rows,cols,6);
% options
%   'GroundTruth',
%   'LikelihoodTargetStateExplored',
%   'FrontierWpts',
%   'LikelihoodOccupancyGraphExplored',
%   'CellAge',
%   'CellState',
%   'KrigingInterp'
spTypes = {'LikelihoodOccupancyGraphExplored','MutualInfo','Entropy','PriorP','PriorQ','PredictedNodes'}; %,'PriorP','PriorR','PredictedNodes'};

playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);
end
