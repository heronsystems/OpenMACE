% This function is rewritten for the implementation of parfor for the
% Monte Carlo simulation

% Sheng Cheng and Artur Wolek, 2019
function [t,entropyHist,totalEntropy,cellMsmtHist,cellStateHist,cellDetHist,numViews,discoveredNodePercentage,mapPercentage] = wrapUpVariables(temp);

for i = 1:1:length(temp.swarmWorldHist)
    t(i) = temp.swarmWorldHist{i}.time;
    %nodeDensityEstimate(i) = swarmWorldHist{i}.nodeDensityEstimate;
    % compute entropy of entire search grid
    entropyHist{i} = temp.swarmWorldHist{i}.entropyMat;
    totalEntropy(i) = temp.swarmWorldHist{i}.totalEntropy;
    
    %
    cellMsmtHist{i} = temp.swarmWorldHist{i}.cellMsmtMat;
    cellStateHist{i} = temp.swarmWorldHist{i}.cellStateMat;
    cellDetHist{i} = temp.swarmWorldHist{i}.cellDetMat;
    numViews(i) = temp.swarmWorldHist{i}.numViews;
    % compute the percentage of discovered nodes
    discoveredNodePercentage(i) = 100*numnodes(temp.swarmWorldHist{i}.exploredGraph)/numnodes(temp.trueWorld.G_env);
end
mapPercentage = temp.swarmWorldHist{end}.mapPercentage;