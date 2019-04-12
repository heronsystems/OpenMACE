function movie_f3_frontierWpts( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figure;
rows = 2;
cols =2;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,2);
spArray(3) = subplot(rows,cols,3);
spArray(4) = subplot(rows,cols,4);

% options
%   'groundTruth',
%   'likelihoodTargetStateExplored',
%   'frontierWpts',
%   'likelihoodOccupancyGraphExplored',
%   'cellAge',
%   'cellState',
%   'krigingInterp'
spTypes = {'LikelihoodTargetStateExplored', 'LikelihoodOccupancyGraphExplored','GroundTruth','FrontierWpts'};
playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);
end