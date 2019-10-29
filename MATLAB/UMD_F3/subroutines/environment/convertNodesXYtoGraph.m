function [G,A,nodeX,nodeY] = convertNodesXYtoGraph(nodesXY, borderOffset, maxDistConnectivity)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [G,A,nodeX,nodeY,xpoly,ypoly] = loadMap(nodesXY, borderOffset, maxDistConnectivity)
%
% Inputs
%   nodesXY : (N x 2) matrix of node locations
%       nodesXY(:,1) : x values
%       nodesXY(:,2) : y values
%   borderOffset : determines how much padding to add to rectangular
%                  boundary
%   maxDistConnectivity : threshold distance used to declare an edge
%                         between two nodes
%
% Output
%   (G, A) : default matlab graph and adjacency matrices 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Build environment graph...')

% create the distance matrix
N = length(nodesXY(:,1));
[xx,yy] = meshgrid(nodesXY(:,1),nodesXY(:,2));
% last term adds a large factor to prevent self-loops
dsq = (xx-xx').^2 + (yy-yy').^2 + eye(N,N)*maxDistConnectivity*maxDistConnectivity*100;
% find nodes with edges
[i,j] = find(dsq <= maxDistConnectivity*maxDistConnectivity);
% define adjacency matrix
A = sparse(i,j,1,N,N);
nodeProps = table(nodesXY(:,1),nodesXY(:,2),'VariableNames',{'x','y'});
% obtain graph
G = graph(A,nodeProps);


nodeX = nodesXY(:,1);
nodeY = nodesXY(:,2);



end
