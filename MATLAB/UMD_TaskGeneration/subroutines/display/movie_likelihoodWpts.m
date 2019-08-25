function movie_likelihoodWpts( swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )
figure;
rows = 2;
cols = 1;
spArray(1) = subplot(rows,cols,1);
spArray(2) = subplot(rows,cols,2);

spTypes = {'GroundTruth','LikelihoodWpts'};
playMovie(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel,spArray, spTypes);
end