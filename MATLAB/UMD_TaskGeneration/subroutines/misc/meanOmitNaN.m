function [means] = meanOmitNaN(data,dir)


if dir == 1 % compute mean of each column, output row vector
    numPts = size(data,1);
    numTrials = size(data,2);
    % remove NaN
    for i = 1:1:numTrials
        dataClean = [];
        for j = 1:1:numPts            
            datum = data(j,i);
            if ( ~isnan( datum ) )
                dataClean = [dataClean datum];
            end
        end
        means(1,i) = mean(dataClean);
    end
    
elseif dir == 2
    numPts = size(data,2);
    numTrials = size(data,1);
    % remove NaN
    for i = 1:1:numTrials
        dataClean = [];
        for j = 1:1:numPts            
            datum = data(i,j);
            if ( ~isnan( datum ) )
                dataClean = [dataClean datum];
            end
        end
        means(i,1) = mean(dataClean);
    end
else
    error('Error: Incorrect dir');
end