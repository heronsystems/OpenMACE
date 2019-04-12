function V = findNodesInViewExploredGraphNoisyMap(exploredGraph, x, y, R, xcp, ycp)
% returns node indices V
N = numnodes(exploredGraph);
V = [];
Rsq = R*R;
for i = 1:1:N 
    dx = xcp( exploredGraph.Nodes.bx( i ) ) - x; 
    dy = ycp( exploredGraph.Nodes.by( i ) ) - y;
    dsq = dx*dx + dy*dy;
    if ( dsq <= Rsq )
       V = [V; i]; 
    end
end

end