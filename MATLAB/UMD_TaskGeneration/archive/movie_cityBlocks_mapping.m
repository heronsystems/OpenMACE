function movie_cityBlocks_mapping( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figure;
rows = 1;
cols = 2;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,2);
% spArray(3) = subplot(rows,cols,3);
% spArray(4) = subplot(rows,cols,4);

% options
%   'groundTruth',
%   'likelihoodTargetStateExplored',
%   'frontierWpts',
%   'likelihoodOccupancyGraphExplored',
%   'cellAge',
%   'cellState',
%   'krigingInterp'
% 
% spTypes = {'likelihoodOccupancyGraphExplored','likelihoodTargetStateExplored'}; %Artur
% spTypes = {'groundTruth','frontierWpts','frontierWpts2','frontierWpts3'};
spTypes = {'GroundTruth','FrontierWpts'};%,'frontierWpts2','frontierWpts3'};% Sheng
playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);
end