function [V,U,O] = updateUnexploredPrior(cellStateMat, xx, yy, numNodesExploredGraph, probAbsent, V, U, O, edgeDir)

% compute adaptive prior for an unexplored cell based on node density
% this becomes the floor
%[nodeDensityEstimate] = exploredAreaNodeDensity(cellStateMat);
%voidDensityEstimate = 1 - nodeDensityEstimate;

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
        if ( cellStateMat(i,j) ~= 2 ) % cellStateMat(j,i) == 1 )
            measurements(k,1) = j;
            measurements(k,2) = i;
            measurements(k,3) = V(i,j);
            k = k + 1;
        end
    end
end

% % debug: plot measurements
% figure; plot(measurements(:,3) ,'ko-')
% hold on;
% plot([1 length(measurements)],[1 1]*voidDensityEstimate, 'r--');

if ( ~isempty(measurements) )
    
    
    %     forecast = ordinaryKrig(xx,yy,measurements,krigingSigma);
    %     forecast(forecast > 1) = 1;
    %     forecast(forecast < 0) = 0;
    
    %%
    inc = 3;
    %nbins = floor(180/inc);
    
    %histogram(edgeDir*180/pi,nbins)
    npeaks = 4;
    ax = 12;
    ay = 0.25;
    
    %
    binEdges = [0:inc:180-inc];
    theta = diff(binEdges)/2+binEdges(1:end-1);
    if isempty(edgeDir)       
        peaks = linspace(0,pi,npeaks+1);
        peaks = peaks(1:end-1);
        weights = ones(1,npeaks);
        disp('Graph is empty');
    else
       

        
        %
        %sep = inc;
        [N,~] = histcounts(edgeDir*180/pi,binEdges);        
        [peaks,weights] = findPeaks(N, npeaks);
        peaks = theta(peaks)*pi/180;
        %peakVal = N(peaks);
        if (min(weights) == 0)
            error('Error with weight computation');
        end
    end
    
     forecast = anisotropicAdaptiveKrigFast2(measurements, ax,ay, peaks, weights, xcp, ycp);
%     peaks*180/pi
%     weights
%     figure;
%     subplot(1,2,1)
%     imagesc(forecast,'XData', xcp, 'YData', ycp); hold on;
%     set(gca,'YDir','Normal');    
%     axis equal;
%     title('Adaptive (4-Peak) Anisotropic Kriging')
%     set(gca,'FontSize',16)
%     xlabel('X')
%     ylabel('Y')
%     colorbar;
%     
%     subplot(1,2,2);
%     data = ones(numel(xcp),numel(ycp))*NaN;
%     for i = 1:1:size(measurements,1)
%         data(measurements(i,2),measurements(i,1)) = measurements(i,3);
%     end
%     
%     imagesc(data,'XData', xcp, 'YData', ycp,'AlphaData',~isnan(data)); hold on;
%     colorbar;
%     set(gca,'YDir','Normal');    
%     axis equal;
%     title('Input data')
%     set(gca,'FontSize',16)
%     xlabel('X')
%     ylabel('Y')
%     pause;
%     close all;


    % determine unexplored area
    [rows, cols] = find(cellStateMat == 2);
    
    % used for prior of target
    numUnexploredCells = numBinsX*numBinsY - numNodesExploredGraph;
    factor = (1 - probAbsent)/numUnexploredCells;
    
    % assign prior
    for i = 1:1:length(rows)
        r = rows(i);
        c = cols(i);
        V(r,c) = forecast(r,c);
        % error check
        if ( V(r,c) < 0 || V(r,c) > 1)
            error('V(r,c) is not a valid probability');
        end
        O(r,c) = (1 - V(r,c))*factor;
        U(r,c) = 1 - O(r,c) - V(r,c);
    end
end

end