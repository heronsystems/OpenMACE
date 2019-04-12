function [G, A, D] = computeConnectivity(G, maxDist)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [G, A, D] = computeConnectivity(G)
%
% Inputs
%   G : cell array defining a graph 
%       G{i}.XY : returns the 2D coordinate corresponding to vertex i
%   maxDist : maximum distance for which an edge exists
%
% Output
%	G{i}.E : gives list of edges (indices of G) to which vertex i is
%           connected
%   A : is the adjacency matrix i.e., A(i,j) indicates vertex i is
%   connected to vertex j
%   D : is the degree matrx i.e., all off-diagonal entries are zero,
%   diagonal entries represent sum of corresponding row of A matrix
%   D(i,j) == sum( A(i,:) ) if i == j , D(i,j) = 0 otherwise
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = length(G);
for i = 1:1:N
    G{i}.E = [];
    for j = 1:1:N
        pti = G{i}.XY;
        ptj = G{j}.XY;
        if norm( pti - ptj ) <= maxDist && (i ~= j)
            A(i,j) = 1;
            G{i}.E = [G{i}.E j];
        else
            A(i,j) = 0;
        end
    end
end

D = zeros(N,N);
for i = 1:1:N
    for j = 1:1:N
        if i == j
           D(i,j) = sum(A(i,:)); 
        end
    end
end

end