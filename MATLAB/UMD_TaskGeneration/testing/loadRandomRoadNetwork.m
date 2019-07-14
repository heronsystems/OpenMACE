function [nodeXY] = loadRandomRoadNetwork(nodeFile, edgeFile, dx, f3Workspace)

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
        f3LowerLeftCornerY = -11;
end

% nodeData = csvread('../external/road-network/node-list');
% edgeData = csvread('../external/road-network/edge-list');
nodeData = csvread(nodeFile);
edgeData = csvread(edgeFile);

maxNode = max(nodeData(:,1));
nodeMat = zeros(maxNode,3);
for i = 1:1:length(nodeData)
   node = nodeData(i,1);
   nodeMat(node,:) =  nodeData(i,:);
end



nodesXY = [];
% figure;
% plot(nodeData(:,2), nodeData(:,3),'o'); hold on;
for i = 1:1:length(edgeData)
   head = nodeMat(edgeData(i,1),2:3);
   tail = nodeMat(edgeData(i,2),2:3);
%    plot([head(1) tail(1)],[head(2) tail(2)],'k-');
    [x,y] = resampleCurve( [head(1) tail(1)],[head(2) tail(2)], dx);
    nodesXY = [ nodesXY; x y ];   
    
end
axis equal;

% resample and snap to grid of width dx
nodesXY(:,1) = round(nodesXY(:,1)./dx)*dx+0.5*dx;
nodesXY(:,2) = round(nodesXY(:,2)./dx)*dx+0.5*dx;

% remove duplicates
nodesXY = unique(nodesXY,'rows');

% remove edgless nodes
edglessNode = [];
edgeFactor = 1;
for i = 1:1:length(nodesXY)
    % calculate distance to all other nodes
    dvec = (nodesXY(i,:) - nodesXY);
    d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
    ind = find(d <= edgeFactor*dx);
    if ( length(ind) == 1 )
        edglessNode = [edglessNode i];
    end
end
nodesXY(edglessNode,:) = [];

% plot( nodesXY(:,1), nodesXY(:,2) ,'mx');
% 
% % plot grid
% dim = 9;
% drawgrid(0,dim/dx,0,dim/dx,dx)
% hold on;
% axis equal;
% shift 
nodesXY(:,1) = f3LowerLeftCornerX + nodesXY(:,1);
nodesXY(:,2) = f3LowerLeftCornerY + nodesXY(:,2);

%end