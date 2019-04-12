function [P,Q,R, mutualInfoSurface] = cellStatePriorAndMutualInfo(cellStateMat, predictedNodeProbMat, ...
    bin2NodeID, exploredGraph, likelihood, sensorSensitvity, sensorDiscretizationLevels, nodeDensityEstimate, probAbsent)

% initialize
numRows = size(cellStateMat,1); % rows
numCols = size(cellStateMat,2); % cols
mutualInfoSurface = zeros(numRows,numCols); % initialize
P = zeros(numRows,numCols); % initialize
Q = P;
R = P;

% iterate through each grid cell
for i = 1:1:numRows
    for j = 1:1:numCols
        switch cellStateMat(i,j)
            case 2 % if cell is unexplored, use interpolated cellStateMat to define prior                
                P(i,j) = nodeDensityEstimate*(1-predictedNodeProbMat(i,j)); 
                numUnexploredCells = numRows*numCols - numnodes(exploredGraph);
                factor = (1 - probAbsent)/numUnexploredCells;
                R(i,j) = (1 - P(i,j))*factor;
                Q(i,j) = 1 - R(i,j) - P(i,j);
            case 1 % if cell is explored and is a node then assign prior based on likelihood
                P(i,j) = 0;
                % map bin i,j to node ID
                trueGraphNodeID = bin2NodeID(i,j);
                % find explored graph node ID
                exploredGraphNodeID = find( exploredGraph.Nodes.trueGraphIndex == trueGraphNodeID );
                % find the node in the
                L = exp(likelihood(exploredGraphNodeID));
                Q(i,j) = 1/(L+1); % prob no target
                R(i,j) = L/(L+1); % prob occupied 
            case 0 % if cell is explored and not a node then assign constant mutual info.
                P(i,j) = 1;
                Q(i,j) = 0;
                R(i,j) = 0;
        end
        % compute mutual information
        mutualInfoSurface(i,j) = mutualInformation(sensorSensitvity, P(i,j), Q(i,j), R(i,j), sensorDiscretizationLevels);
    end
end