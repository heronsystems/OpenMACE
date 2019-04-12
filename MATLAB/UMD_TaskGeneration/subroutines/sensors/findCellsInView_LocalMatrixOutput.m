function [reachedCell] = findCellsInView_LocalMatrixOutput(agentX,agentY,mutualInfoSurface,xcp,ycp,Rsense)

% binX and binY is the nominal bin in which the vehicle currently is in
%agent = [xcp(agentBinX) ycp(agentBinY)];
agent = [agentX agentY];
numBinsX = length(xcp);
numBinsY = length(ycp);
dx = xcp(2) - xcp(1);
agentBinX = max(round( (agentX - xcp(1)) ./  dx ) + 1,1);
agentBinY = max(round( (agentY - ycp(1)) ./  dx ) + 1,1);
agentBinX = min(agentBinX, numBinsX);
agentBinY = min(agentBinY, numBinsY);

reachedCell = zeros(size(mutualInfoSurface));

windowWidth = 3*ceil(Rsense/dx)+1;
halfWidth = floor((windowWidth-1)/2);
% determine the bin indices that need to be checked (i.e., that define
% the current window)
binXStart = max( agentBinX - halfWidth + 1, 1);
binXEnd = min( agentBinX + halfWidth + 1, numBinsX );
binYStart = max( agentBinY - halfWidth + 1, 1);
binYEnd = min( agentBinY + halfWidth + 1, numBinsY );
% create search bin vectors to search locally around each agent
for bx = [binXStart:1:binXEnd]
    for by = [binYStart:1:binYEnd]
        controlPt = [xcp(bx) ycp(by)];
        % check if control point is within sensing range
        if ( norm(controlPt-agent) <= Rsense )
            reachedCell(by,bx) = 1;
        end
    end
end

% Artur: Was having issue with this code on RandalsAtF3 Map. Rsense is not
% necessarily an integer so cannot use it for k and j limits calculation
% numBinsX = length(xcp);
% numBinsY = length(ycp);
% for k = max(1,cellCoordinates(1)-Rsense):min(size(mutualInfoSurface,1),cellCoordinates(1)+Rsense)
%     for j = max(1,cellCoordinates(2)-Rsense):min(size(mutualInfoSurface,2),cellCoordinates(2)+Rsense)
%         if sqrt((k-cellCoordinates(1))^2 + (j-cellCoordinates(2))^2) <= Rsense
%             reachedCell(j,k) = 1;
%         end
%     end
% end
