function movie_f3_lrdt( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figure;
rows = 3;
cols = 1;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,2);
spArray(3) = subplot(rows,cols,3);
% options
%   'groundTruth',
%   'likelihoodTargetStateExplored',
%   'frontierWpts',
%   'likelihoodOccupancyGraphExplored',
%   'cellAge',
%   'cellState',
%   'krigingInterp'
% 
spTypes = {'GroundTruth','LikelihoodOccupancyGraphExplored','LikelihoodTargetStateExplored'}; %Artur
playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);




end