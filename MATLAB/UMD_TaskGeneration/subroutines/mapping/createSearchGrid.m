function [xControlPts,yControlPts,numNodesMat,bin2NodeID] = createSearchGrid(minX, maxX, minY, maxY, nodeX, nodeY, numBinsX, numBinsY)
% Input:
% (xpoly, ypoly) : boundaries of search area used to define grid
% (nodeX, nodeY) : locations of nodes
% (numBinsX, numBinsY) : gives level of discretization

% Output: 
%   (xcp, ycp) : control pts corresponding to the center of each bin
%   numNodesMat : gives number of nodes in each bin
%   bin2NodeID : (numBinsY x numBinsX) matrix whose entry is the
%       index of the node (if exists) or 0 (if no node)
%   Note: node indices are assigned in the order they are encountered in
%         (nodeX, nodeY)

% boundaries and min/max values
% maxX = max(xpoly);
% maxY = max(ypoly);
% minX = min(xpoly);
% minY = min(ypoly);

% spacing
dx = (maxX - minX)/(numBinsX);
dy = (maxY - minY)/(numBinsY);

% control points
xControlPts = [dx/2:dx:(dx)*(numBinsX)] + minX;
yControlPts = [dx/2:dy:(dy)*(numBinsY)] + minY;

% initialize
N = length(nodeX); 
numNodesMat = zeros(numBinsY, numBinsX);
bin2NodeID = zeros(numBinsY, numBinsX);

for i = 1:1:N
    % determine bin
    binX = interp1(xControlPts, [1:1:numBinsX], nodeX(i),'nearest','extrap');
    binY = interp1(yControlPts, [1:1:numBinsY], nodeY(i),'nearest','extrap');
    % check bins are valid
    if ( binX < 1 || binX > numBinsX )
       error('creatSampleNeedGrid: binX out of range'); 
    elseif ( binY < 1 || binY > numBinsY )
       error('creatSampleNeedGrid: binY out of range');  
    end
    % increment 
    numNodesMat(binY,binX) = numNodesMat(binY,binX) + 1;
    % record the coordinate of this node in cptNodesMat
    bin2NodeID(binY,binX) = i;
end

% check if bin2NodeID contains all of the nodes
flattenBin2NodeID = reshape(bin2NodeID,numBinsY*numBinsX,1);
for i = 1:1:N
    if ( isempty(find( flattenBin2NodeID == i ) ) )
        fprintf('Warning, Node %i not found in bin2NodeID matrix!\n', i);        
    end
end


