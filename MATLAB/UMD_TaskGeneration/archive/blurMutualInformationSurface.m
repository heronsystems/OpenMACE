function mutualInfoSurface = blurMutualInformationSurface( mutualInfoSurface, xcp, ycp, searchAreaGridX, searchAreaGridY, krigingSigma)
% initialize
numRows = size(searchAreaGridX,1); % rows
numCols = size(searchAreaGridX,2); % cols
measurements = [];
minInfo = min(min(mutualInfoSurface));
maxInfo = max(max(mutualInfoSurface));
% compute measurements (all non-zero mutual information matrices
k = 1;
for i = 1:1:numRows
    for j = 1:1:numCols
        if (mutualInfoSurface(i,j) > 0)
            measurements(k,1) = xcp(i); % x position
            measurements(k,2) = ycp(j); % y position
            measurements(k,3) = mutualInfoSurface(i,j);
            k = k + 1;
        end
    end
end
% perform the blur
mutualInfoSurface = krigInterp(searchAreaGridX, searchAreaGridY, measurements, krigingSigma);
% the kriging interpolation does not preserve magnitude so we re-scale
mutualInfoSurface = normalizeMatrix(mutualInfoSurface, minInfo, maxInfo);
end