function nodesXY = loadCityBlocksNodes_atF3(blockLength, numBlocks, L, f3Workspace)
% Input
%   blockLength : length of a block
%   L : distance between nodes
%   numBocks : number of blocks (both NS and EW)
%   buffer: 
%   f3Workspace : flag determining where to shift cityblocks
% Output:
%   nodesXY : (N x 2) matrix of node locations
%   nodesXY(:,1) : x values
%   nodesXY(:,2) : y values

% hardcoded parameters
switch f3Workspace
    case 'full'
        f3Width = 26;
        f3Length = (59+28);
        f3LowerLeftCornerX = -59;
        f3LowerLeftCornerY = -13;
    case 'right-square'
        f3Width = 26;
        f3Length = 28;
        f3LowerLeftCornerX = 5;
        f3LowerLeftCornerY = -10; % -11;
end

% a temporary vector used to define node locations
base = [L/2:L:blockLength*numBlocks+L/2]';
nodesXY = [];
% build E/W streets
for i = 1:1:numBlocks+1
    nodesXY = [nodesXY; base, (i-1)*blockLength.*ones(size(base))+L/2 ];
end
% build N/S streets
for i = 1:1:numBlocks+1
    nodesXY = [nodesXY; (i-1)*blockLength.*ones(size(base))+L/2, base ];
end

% shift 
nodesXY(:,1) = f3LowerLeftCornerX + nodesXY(:,1);
nodesXY(:,2) = f3LowerLeftCornerY + nodesXY(:,2);

% remove duplicate intersections
nodesXY = unique(nodesXY,'rows');

end