function [mutualInfoSurface, P, Q, R, predictedNodeProbMat, nodeDensityEstimate, entropyMat, totalEntropy] = mutualInformationSurface( ...
    maxUnexploredPrior, sensorSensitvity, sensorDiscretizationLevels, ...
    krigingSigma, searchAreaGridX, searchAreaGridY, bin2NodeID, ...
    exploredGraph, likelihood, cellStateMat, probAbsent )


% predict new nodes
[predictedNodeProbMat, nodeDensityEstimate] = predictNewNodes(cellStateMat, searchAreaGridX, searchAreaGridY, maxUnexploredPrior, krigingSigma);

% compute mutual information surface (pixelized)
%   this uses predicted nodes, unexplored area, and likelihood
[P,Q,R, mutualInfoSurface] = cellStatePriorAndMutualInfo(cellStateMat, predictedNodeProbMat, ...
    bin2NodeID, exploredGraph, likelihood, sensorSensitvity, sensorDiscretizationLevels,nodeDensityEstimate, probAbsent);

%
[entropyMat, totalEntropy] = entropyMatrix(P, Q, R);

end
