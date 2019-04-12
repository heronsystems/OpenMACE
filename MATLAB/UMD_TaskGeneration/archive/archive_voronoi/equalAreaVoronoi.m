% equalAreaVoronoi

function [xCM, yCM, cellMass, Ac] = equalAreaVoronoi(xx, yy, zz, numCells, xs, ys, maxIters, percentTol, plotFlag)

% pg. 86:4
% Balzer, M., Schlömer, T., & Deussen, O. (2009). Capacity-constrained 
% point distributions. ACM Transactions on Graphics, 28(3), 1. 
% https://doi.org/10.1145/1531326.1531392

% step 2
[A, xg, yg, cellArea, Ac, xCM, yCM, cellMass]  = capacityConstrainedVoronoi(xx, yy, zz, numCells, xs, ys, plotFlag);

plotFlag = 1;
% check tolerance criteria
i = 1;
meanM = mean(cellMass)
maxOffsetSqPercent = max((cellMass - meanM).^2 / meanM^2)
cellArea

while ( maxOffsetSqPercent > percentTol && i < maxIters )
    [A, xg, yg, cellArea, Ac, xCM, yCM, cellMass] = capacityConstrainedVoronoi(xx, yy, zz, numCells, xCM, yCM, plotFlag, Ac);
    cellArea
    i = i + 1
    meanM = mean(cellMass);
    disp('-----')
    maxOffsetSqPercent = max((cellMass - meanM).^2 / meanM^2)
    cellMass
end