function [nodesXY, LatRef, LongRef, G, A ] = loadOpenStreetMapNodesFlex(fileName, refX, refY, boxlength, boxwidth, angle, dx, removeList)

% parse baseline map
[ways, LatRef, LongRef] = parseOpenStreetMap(fileName);

% generate box
boxXY = boxCorners(refX,refY,boxlength,boxwidth,angle);

% clip, shift to origin, and scale
shiftX = -refX;
shiftY = -refY;
waysmod = manipOpenStreetMap( ways, boxXY, -angle, 1, shiftX, shiftY );

% define grid
numX = floor(boxlength/dx);
numY = floor(boxwidth/dx);

% remove some ways
j = 1;
for i = 1:1:length(waysmod)
    if ( ~any(i==removeList) )
        waysmodtrunc{j} = waysmod{i};
        j = j + 1;
    end
end

% init
bin2NodeID = zeros(numY, numX);
nodesXY = [];
A = [];
numNodes = 0;

% go through each ways and resample and snap to grid of width dx
for i = 1:1:length(waysmodtrunc)
    % resample curve with many points per bin
    [x,y] = resampleCurve( waysmodtrunc{i}(:,1), waysmodtrunc{i}(:,2), sqrt(2)*dx/100);
    % find bin location of each sample
    [bins] = curveToBins(x,y,dx,numX, numY);
    binNum = bins(:,3); % binNum is index of bin for each (x,y) pt on curve
    curBin = binNum(1);
    binChange = [];
    maxBinNum = numX*numY;
    % bins are determined differently depending on if edge starts on or off
    % or in middle of gridded area
    % binChange : index along resampled curve when the bin number changes
    for j = 1:1:length(binNum)
        % when curBin is NaN (outside grid) but jth bin is not:
        % i.e., entering the grid
        if ( isnan(curBin) && ~isnan(binNum(j)) )
            binChange = [binChange j];
            fprintf('First bin found on step %d \n',j);
            % special case: j = 1 and is not NaN
        elseif ( j == 1 && ~isnan(binNum(j)) )
            binChange = 1;
        end
        % nominal path (inside grid): jth bin is different than currentBin
        if ( binNum(j) ~= curBin && ~isnan(binNum(j)) )
            binChange = [binChange j];
        end
        % when exiting (outside grid): jth bin is NaN but curBin is not
        if ( ~isnan(curBin) && isnan(binNum(j)) )
            binChange = [binChange j];
        end
        % special case: j is last index and both curBin and j are not NaN
        if ( ~isnan(curBin) && j == length(binNum) && ~isnan(binNum(j)))
            binChange = [binChange j];
        end
        
        %         % error checking
        %         if ( ~isempty(binChange) )
        %            lastBin = binNum(binChange(end));
        %             if ( lastBin > maxBinNum || lastBin <= 0 || isnan(lastBin) )
        %                error('Invalid Bin Change assigned.');
        %             end
        %         end
        curBin = binNum(j);
    end
    % assign node at middle range of each bin
    edgeList = [];    
    k = 1; % number of segments
    for j = 1:1:length(binChange)-1
        % special case: if a curve goes into and outside of the grid then
        % the midpoint between these bin changes is outside the box
        if ( ~isnan( binNum(binChange(j)) ) && ~isnan( binNum(binChange(j+1)) ) )
            % index along (x,y) curve in middle of bin changes
            midInd = floor((binChange(j)+binChange(j+1))/2);
            nodesXY = [nodesXY; x(midInd) y(midInd)];
            numNodes = numNodes + 1;
            if isempty(edgeList)
                edgeList(k,1) = numNodes;
            else
                edgeList(k,end+1) = numNodes;
            end
            node2bin(numNodes) = bins(midInd,3);
            % bin2NodeID is a matrix indicating node ID
            bin2NodeID( bins(midInd,1) , bins(midInd,2) ) = 1;
            nodeID2bin( numNodes,:) = [bins(midInd,1) , bins(midInd,2)];
        else
            
            k = k + 1;
        end
    end
    for k = 1:1:size(edgeList,1)
    
    for j = 1:1:length(edgeList(k,:))-1
        if ( edgeList(k,j+1)~= 0 && edgeList(k,j)~= 0 )
        A(edgeList(k,j),edgeList(k,j+1)) = 1;
        A(edgeList(k,j+1),edgeList(k,j)) = 1;
        end
    end
    end
end

% if multiple nodes are in one cell, we do:
% - average there locations to create a new node
% - create new edges, one for each node that was previously present

% find unique rows in nodeID2bin
[~, ind] = unique(nodeID2bin, 'rows');
% we must handle any node that is a duplicate
duplicate_ind = setdiff(1:size(nodeID2bin, 1), ind);
nodesToDelete = [];
for i = 1:1:length(duplicate_ind)
    % find set of all nodes in this bin
    commonBinNodes = find(ismember(nodeID2bin, nodeID2bin(duplicate_ind(i),:),'rows')==1);
    % get list of (other) nodes that are connected to each common node
    edgeList = [];
    for j = 1:1:length(commonBinNodes)
        curNodeEdges = find(A(commonBinNodes(j),:) == 1);
        edgeList = [edgeList curNodeEdges];
    end
    % add new node to nodeXY list
    newNodeX = mean( nodesXY(commonBinNodes,1) );
    newNodeY = mean( nodesXY(commonBinNodes,2) );
    nodesXY(end+1,:) = [newNodeX newNodeY];
    A(end+1,:) = zeros(1,size(A,2));
    A(:,end+1) = zeros(size(A,1),1);
    % add new row/col to adjacency matrix
    for j = 1:1:length(edgeList)
        A(end,edgeList(j)) = 1;
        A(edgeList(j),end) = 1;
    end
    nodesToDelete = [nodesToDelete; commonBinNodes];
end


% modify matrix to delete nodes
nodesXY(nodesToDelete,:) = [];
A(nodesToDelete,:) = [];
A(:,nodesToDelete) = [];

nodeProps = table(nodesXY(:,1),nodesXY(:,2),'VariableNames',{'x','y'});
G = graph(A,nodeProps);


plotFlag = 1;
if (plotFlag)
    % figure;
    figure;
    imagesc(bin2NodeID)
    
    % plot ways with labels
    for i = 1:1:length(waysmod)
        plot(waysmod{i}(:,1), waysmod{i}(:,2) ); hold on;
    end
    % plot ways with labels
    plot(nodesXY(:,1), nodesXY(:,2), 'r+'); hold on;
    % plot grid
    xmin = 0;
    ymin = 0;
    numXpx = floor(boxlength/dx);
    numYpx = floor(boxwidth/dx);
    drawgrid(xmin,numXpx,ymin,numYpx,dx)
    xlim([xmin xmin+numXpx*dx]);
    ylim([ymin ymin+numYpx*dx]);
    
    hold on;
    axis equal;
    
    figure;
    plot(G,'XData',nodesXY(:,1),'YData',nodesXY(:,2),'NodeLabel',[]);hold on;
    drawgrid(xmin,numXpx,ymin,numYpx,dx)
    xlim([xmin xmin+numXpx*dx]);
    ylim([ymin ymin+numYpx*dx]);
end