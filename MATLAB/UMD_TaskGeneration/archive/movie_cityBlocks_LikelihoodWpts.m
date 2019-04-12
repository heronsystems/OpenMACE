function movie_likelihoodWpts( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figure;
rows = 2;
cols = 1;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,2);
%spArray(3) = subplot(rows,cols,[3 4]);
%set(spArray(3), 'OuterPosition', [0,0, 1, .4]);
%spArray(4) = subplot(rows,cols,4);

% options
%   'GroundTruth',
%   'LikelihoodTargetStateExplored',
%   'FrontierWpts',
%   'LikelihoodOccupancyGraphExplored',
%   'CellAge',
%   'CellState',
%   'KrigingInterp'
spTypes = {'GroundTruth','LikelihoodWpts'};
playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);
end