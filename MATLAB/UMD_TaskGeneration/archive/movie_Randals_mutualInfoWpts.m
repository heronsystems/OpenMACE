function movie_Randals_mutualInfoWpts( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figh = figure;
rows = 1;
cols = 3;

% cols = 3;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,2);
spArray(3) = subplot(rows,cols,3);
% 
figh.Position = [0 0 1600 600];
b = 0.25;
w = 0.6;
h = 0.6;
spArray(1).Position = [0.0-0.1 b w h];
spArray(2).Position = [0.3-0.1 b w h];
spArray(3).Position = [0.65-0.1 b w h];


% options
%   'GroundTruth',
%   'LikelihoodTargetStateExplored',
%   'FrontierWpts',
%   'LikelihoodOccupancyGraphExplored',
%   'CellAge',
%   'CellState',
%   'KrigingInterp'
spTypes = {'LikelihoodOccupancyGraphExplored','LikelihoodTargetStateExplored','MutualInfoWpts'};
playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);
end