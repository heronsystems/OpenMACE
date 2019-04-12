function [targetStateSpaceGraph,Mpc2s,Mc,Mp] = addNodeToTargetStateSpace(fromNode, toNode, targetStateSpaceGraph, exploredGraph, trueWorldGraph, Mpc2s,Mc,Mp)

% indices are in the 
toNodeTrueWorld = exploredGraph.Nodes.trueGraphIndex(toNode);
fromNodeTrueWorld = exploredGraph.Nodes.trueGraphIndex(fromNode);

dx = trueWorldGraph.Nodes.x(toNodeTrueWorld) - trueWorldGraph.Nodes.x(fromNodeTrueWorld);
dy = trueWorldGraph.Nodes.y(toNodeTrueWorld) - trueWorldGraph.Nodes.y(fromNodeTrueWorld);
heading = atan2( dy , dx );
NodeProps = table(fromNode,toNode,heading,'VariableNames',{'prevNode','curNode','heading'});
targetStateSpaceGraph = addnode(targetStateSpaceGraph, NodeProps);
Mpc2s(fromNode,toNode) = numnodes(targetStateSpaceGraph);
Mp(fromNode,numnodes(targetStateSpaceGraph)) = 1;
Mc(toNode,numnodes(targetStateSpaceGraph)) = 1;
end