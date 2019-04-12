function [G_env, A_env] = buildEnvGraph( nodesXY, maxDistConnectivity )
% Input:
%   nodesXY : (N x 2) matrix of node locations
%       nodesXY(:,1) : x values
%       nodesXY(:,2) : y values
%   maxDistConnectivity : max value between nodes to define an edge
%
% Output:
%   G_env, A_env  : environment graph and adjacency matrix

% create the distance matrix
N = length(nodesXY(:,1));
[xx,yy] = meshgrid(nodesXY(:,1),nodesXY(:,2));
% last term adds a large factor to prevent self-loops
dsq = (xx-xx').^2 + (yy-yy').^2 + eye(N,N)*maxDistConnectivity*maxDistConnectivity*100;
% find nodes with edges
[i,j] = find(dsq <= maxDistConnectivity*maxDistConnectivity);
% define adjacency matrix
A_env = sparse(i,j,1,N,N);
nodeProps = table(nodesXY(:,1),nodesXY(:,2),'VariableNames',{'x','y'});
% obtain graph
G_env = graph(A_env,nodeProps);

end