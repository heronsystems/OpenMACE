function V = findNodesInView(trueWorldGraph, x, y, R)

N = numnodes(G_env);
V = [];
Rsq = R*R;
for i = 1:1:N 
    dx = trueWorldGraph.Nodes.x(i) - x; 
    dy = trueWorldGraph.Nodes.y(i) - y;
    dsq = dx*dx + dy*dy;
    if ( dsq <= Rsq )
       V = [V; i]; 
    end
end

end
