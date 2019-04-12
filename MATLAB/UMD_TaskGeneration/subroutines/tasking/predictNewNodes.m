function [predictedNodeProbMat, nodeDensityEstimate] = predictNewNodes(cellStateMat, xx, yy, maxUnexploredPrior, krigingSigma, nodeDensityInitGuess)

% interpolate cellStateMat to predict new nodes
xcp = xx(1,:);
ycp = yy(:,1)';
numBinsX = length(xcp);
numBinsY = length(ycp);

% determine which cells have been found to contain nodes
k = 1;
measurements = [];
for i = 1:1:numBinsY
    for j = 1:1:numBinsX
        if ( cellStateMat(i,j) == 1 ) % cellStateMat(j,i) == 1 )
            measurements(k,1) = xcp(j);
            measurements(k,2) = ycp(i);
            measurements(k,3) = 1;
            k = k + 1;
        end
    end
end


if ( isempty(measurements) )
    predictedNodeProbMat = 0*ones(size(cellStateMat));
    nodeDensityEstimate = nodeDensityInitGuess;
else
% performance a linear interpolation
predictedNodeProbMat = krigInterp(xx,yy,measurements,krigingSigma);

% compute adaptive prior for an unexplored cell based on node density
% this becomes the floor
[nodeDensityEstimate] = exploredAreaNodeDensity(cellStateMat);

% scale the interpolated cellStateMat from floor to cieling
% the user defined maximum is the cieling
predictedNodeProbMat = normalizeMatrix(predictedNodeProbMat, nodeDensityEstimate, maxUnexploredPrior);

% make a final pass and mask explored empty cells (set prob to zero) and
% lift up known nodes (set prob to one)
for i = 1:1:numBinsY
    for j = 1:1:numBinsX
        switch cellStateMat(i,j)
            case 1
                predictedNodeProbMat(i,j) = 1;
            case 0
                predictedNodeProbMat(i,j) = 0;
        end
    end
end

end



end