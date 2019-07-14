function [cellsInView] = findCellsInView(agent,xcp,ycp,dx,dy,numBinsX,numBinsY,halfWidth,Rsense)
% Input:
%   
% Output:
%   cellsInView: (bx, by) coordinates of cells in view of current agent
%   discoveredCells: (bx, by) coordinates of cells that have never been
%       visited i.e. cellStateMat(bx,by) == 2
%   nodeCells: (bx, by) coordinates of discovered cells subset which
%       contains nodes
%   cellStateMat: updated cellState matrix (1,2,0) 

% find the nominal bin in which the vehicle currently is in
binX = max(floor( (agent(1) - xcp(1)) /  dx ),1);
binY = max(floor( (agent(2) - ycp(1)) /  dy ),1);
binX = min(binX, numBinsX);
binY = min(binY, numBinsY);
% determine the bin indices that need to be checked (i.e., that define
% the current window)
binXStart = max( binX - halfWidth + 1, 1);
binXEnd = min( binX + halfWidth + 1, numBinsX );
binYStart = max( binY - halfWidth + 1, 1);
binYEnd = min( binY + halfWidth + 1, numBinsY );
% create search bin vectors to search locally around each agent
cellsInView = [];
for bx = [binXStart:1:binXEnd]
    for by = [binYStart:1:binYEnd]
        controlPt = [xcp(bx) ycp(by)];
        % check if control point is within sensing range
        if ( norm(controlPt-agent) <= Rsense )
            cellsInView = [ cellsInView; bx by];           
        end
    end
end
end
