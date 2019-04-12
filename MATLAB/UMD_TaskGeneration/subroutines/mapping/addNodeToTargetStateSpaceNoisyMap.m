function [targetStateSpaceGraph,Mpc2s,Mc,Mp] = addNodeToTargetStateSpaceNoisyMap(fromNode, toNode, targetStateSpaceGraph, exploredGraph, Mpc2s,Mc,Mp,xcp,ycp)

% indices are in the 
bx2 = exploredGraph.Nodes.bx(toNode);
by2 = exploredGraph.Nodes.by(toNode);
bx1 = exploredGraph.Nodes.bx(fromNode);
by1 = exploredGraph.Nodes.by(fromNode);
dx = xcp(bx2) - xcp(bx1);
dy = ycp(by2) - ycp(by1);
heading = atan2( dy , dx );
NodeProps = table(fromNode,toNode,heading,'VariableNames',{'prevNode','curNode','heading'});
targetStateSpaceGraph = addnode(targetStateSpaceGraph, NodeProps);
Mpc2s(fromNode,toNode) = numnodes(targetStateSpaceGraph);
Mp(fromNode,numnodes(targetStateSpaceGraph)) = 1;
Mc(toNode,numnodes(targetStateSpaceGraph)) = 1;
end