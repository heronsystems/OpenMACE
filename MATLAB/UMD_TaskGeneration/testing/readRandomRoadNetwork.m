%function [nodeX,nodeY,edges] = readRandomRoadNetwork(nodeFile, edgeFile)
clear all;
close all;
clc;

nodeData = csvread('../external/road-network/node-list');
edgeData = csvread('../external/road-network/edge-list');

maxNode = max(nodeData(:,1));
nodeMat = zeros(maxNode,3);
for i = 1:1:length(nodeData)
   node = nodeData(i,1);
   nodeMat(node,:) =  nodeData(i,:);
end
figure;
dx = 0.25;
nodesXY = [];
plot(nodeData(:,2), nodeData(:,3),'o'); hold on;
for i = 1:1:length(edgeData)
   head = nodeMat(edgeData(i,1),2:3);
   tail = nodeMat(edgeData(i,2),2:3);
   plot([head(1) tail(1)],[head(2) tail(2)],'k-');
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
plot( nodesXY(:,1), nodesXY(:,2) ,'mx');

% plot grid
dim = 9;
drawgrid(0,dim/dx,0,dim/dx,dx)
hold on;
axis equal;


% load environment 
trueWorld = struct;
trueWorld.type = 'randomRoads'; % 'cityblocks', %'openStreetMap', 'osmAtF3'
trueWorld.f3Workspace = 'right-square'; % 'full', 'right-square'
trueWorld.borderOffset = 0; % used for adding padding to the map
trueWorld.binWidth = 0.5; % distance used to declare two nodes as connected (use 7 for open street map)
trueWorld.folder = './data/'; % folder with map file
trueWorld.buffer = 0;
trueWorld.fileName = 'randomRoads';

% derived world model parameters
trueWorld = loadEnvironment(trueWorld, targetModel);

% given the sensing radius, measurements can only be obtained within some
% window of bins around the target
trueWorld.windowWidth = 3*ceil(swarmModel.Rsense/trueWorld.binWidth)+1;
trueWorld.halfWidth = floor((trueWorld.windowWidth-1)/2);

%end