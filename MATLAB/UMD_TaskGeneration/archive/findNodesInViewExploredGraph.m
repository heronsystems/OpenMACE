function V = findNodesInViewExploredGraph(exploredGraph, trueWorldGraph, x, y, R)
% returns node indices V
N = numnodes(exploredGraph);
V = [];
Rsq = R*R;
for i = 1:1:N 
    dx = trueWorldGraph.Nodes.x( exploredGraph.Nodes.trueGraphIndex(i) ) - x; 
    dy = trueWorldGraph.Nodes.y( exploredGraph.Nodes.trueGraphIndex(i) ) - y;
    dsq = dx*dx + dy*dy;
    if ( dsq <= Rsq )
       V = [V; i]; 
    end
end

end