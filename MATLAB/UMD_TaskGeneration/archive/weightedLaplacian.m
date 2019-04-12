function [L, D, A] = weightedLaplacian(W)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [L, D, A] = weightedLaplacian(W)
%
% Inputs
%   W : a square, N x N matrix representing the edge weights of a graph
%
% Output
%   L : the weighted laplacian
%   D : the degree matrix
%   A : the adjacency matrix
%
% A. Wolek, Aug 2018       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

D = diag( sum( W , 2) );
numRows = size(W,1);
numCols = size(W,2);
A = zeros(numRows, numCols);
for i = 1:1:numRows
    for j = 1:1:numCols
        if (W(i,j) > 0 && i ~= j) 
            A(i,j) = 1;
        end
    end
end
L = D - W;
end