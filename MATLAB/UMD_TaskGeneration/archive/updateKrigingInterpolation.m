function forecastNew = updateKrigingInterpolation(xx, yy, forecastOld, krigingMethod, krigingSigma, cellStateMat, nodeCells)

xcp = xx(1,:);
ycp = yy(:,1)';
numBinsX = length(xcp);
numBinsY = length(ycp);

newMeasurementsMat = zeros(numBinsX,numBinsY);
% treat newly discovered cells as additional measurements of unit value
for i = 1:1:size(nodeCells,1)
    bx = nodeCells(i,1);
    by = nodeCells(i,2);
    newMeasurementsMat(by,bx) = 1;
end

% determine which cells have been found to contain nodes
k = 1;
measurements = [];
for i = 1:1:numBinsY
    for j = 1:1:numBinsX
        switch krigingMethod
            case 'recursive'
                if ( newMeasurementsMat(j,i) == 1 )
                    measurements(k,1) = xcp(i);
                    measurements(k,2) = ycp(j);
                    measurements(k,3) = 1;
                    k = k + 1;
                end
            case 'standard'
                if ( cellStateMat(j,i) == 1 )
                    measurements(k,1) = xcp(i);
                    measurements(k,2) = ycp(j);
                    measurements(k,3) = 1;
                    k = k + 1;
                end
            otherwise
                warning('updateCellStates.m: Warning, krigingMethod invalid.');
        end
    end
end

% performance a linear interpolation
switch krigingMethod
    case 'recursive'
        if (~isempty(measurements))
            forecastNew = recursiveKrigInterp(forecastOld, xx, yy, measurements, krigingSigma);
        else
            forecastNew = forecastOld;
        end
    case 'standard'
        if (~isempty(measurements))
            forecastNew = krigInterp(xx,yy,measurements,krigingSigma);
        else
            forecastNew = forecastOld;
        end
    otherwise
        warning('updateSwarmWorld.m: Warning, krigingMethod invalid.');
end

end