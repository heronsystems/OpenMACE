function [Gnew] = truncateGraph(G, xmin, xmax, ymin, ymax)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [Gnew] = truncateGraph(G, xmin, xmax, ymin, ymax)
%
% Inputs
%   G : cell array defining a graph 
%       G{i}.XY : returns the 2D coordinate corresponding to vertex 
%   (xmin, xmax) : x values defining boundaries of a new truncated graph
%   (ymin, ymax) : y values  "--"
%
% Output
%   Gnew: the new truncated graph
%
% A. Wolek, Sep 2018       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
j = 1;
for i = 1:1:length(G)
    x = G{i}.XY(1);
    y = G{i}.XY(2);    
    if ( x >= xmin && x <= xmax && y >= ymin && y <= ymax )
        Gnew{j}.XY = [ x y ]';
        j = j + 1;
    end
end
end