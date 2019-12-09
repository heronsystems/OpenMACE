function [V,U,O, mutualInfoSurface] = mutualInfoMappingTarget(cellStateMat, predictedNodeProbMat, ...
    bin2NodeID, exploredGraph, likelihood, sensorSensitvity, sensorDiscretizationLevels, nodeDensityEstimate, probAbsent)

% initialize
numRows = size(cellStateMat,1); % rows
numCols = size(cellStateMat,2); % cols
mutualInfoSurface = zeros(numRows,numCols); % initialize
V = zeros(numRows,numCols); % initialize
U = V;
O = V;

% iterate through each grid cell
for i = 1:1:numRows
    for j = 1:1:numCols
        % compute mutual information
        mutualInfoSurface(i,j) = mutualInformation(sensorSensitvity, V(i,j), U(i,j), O(i,j), sensorDiscretizationLevels);
    end
end