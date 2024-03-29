function [cellsInView,cellStateMat,discoveredCells,nodeCells] = gridCellSensorFunction(agent,xcp,ycp,dx,dy,numBinsX,numBinsY,halfWidth,Rsense,numNodesMat,cellsInView,cellStateMat,discoveredCells,nodeCells)
% gridCellSensorFunction
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
for bx = [binXStart:1:binXEnd]
    for by = [binYStart:1:binYEnd]
        controlPt = [xcp(bx) ycp(by)];
        % check if control point is within sensing range
        if ( norm(controlPt-agent) <= Rsense )
            cellsInView = [ cellsInView; bx by];
            % if cellStateMat entry = 2 this it has never been visited
            if ( cellStateMat(by,bx) == 2 )
                discoveredCells = [ discoveredCells; bx by];
                % update the cell state mat:
                % 1 - if it contains a node of the occupancy graph
                % 0 - if it is an obstacle cell
                cellStateMat(by,bx) = (numNodesMat(by,bx)~= 0);  %
                if ( cellStateMat(by,bx) == 1 )
                    % determine index of this node in the trueWorld
                    nodeCells = [ nodeCells; bx by];                        
                end
            end
        end
    end
end
end