function [nodes] = nodesInFOV(x,y,nodeX,nodeY,R)
    % compute distance sq to each node 
    d2 = (nodeX - x).^2 + (nodeY - y).^2;
    % find nodes within distance R 
    nodes = find( d2 <= R^2 );
end