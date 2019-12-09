function [nodeDensityEstimate] = exploredAreaNodeDensity(cellStateMat)
% Recall the cell state mat has entries:
% 2 - initialized value (unexplored)
% 1 - if it is explored and contains a node of the occupancy graph
% 0 - if it is explored and does not contain a node

% here we want to compute what percentage of the explored area is state 1

numRows = size(cellStateMat,1); % rows
numCols = size(cellStateMat,2); % cols
numCells = numRows*numCols; % number of cells

numNodes = 0;
%numEmpty = 0;
numUnexplored = 0;
for i = 1:1:numRows    
    numNodes = numNodes + length(find(cellStateMat(i,:)==1));
    %numEmpty = numEmpty + length(find(A(1,:)==0));
    numUnexplored = numUnexplored + length(find(cellStateMat(i,:)==2));
end

if ( numUnexplored == numCells )
   nodeDensityEstimate = 1/3;
else
   nodeDensityEstimate = numNodes/(numCells-numUnexplored);
end


end