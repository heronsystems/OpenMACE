function [peaks,weights] = findPeaks(data, numPeaks)
N = length(data);
peaks = zeros(numPeaks,1);
weights = zeros(numPeaks,1);
for i = 1:1:numPeaks
    [~,maxInd] = max(data);
    peaks(i) = maxInd;
    if (maxInd == 1)
        weights(i) = data(end) + data(1) + data(2);
        data(end) = 0;
        data(1:2) = 0;        
    elseif (maxInd == N)
        weights(i) = data(end) + data(1) + data(2);
        data(N-1:N) = 0;
        data(1) = 0;
    else
        weights(i) = sum(data(maxInd-1:maxInd+1));
        data(maxInd-1:maxInd+1) = 0;
    end
end

% remove peaks with zero weight
peaks( weights == 0 ) = [];
weights( weights == 0 ) = [];
end