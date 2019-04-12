function swarmWorld = updateTargetStateSpaceGraph(swarmWorld, trueWorld, curNode, prevNode )

% get ID of current and previous node in true world graph 
curNodeTrueWorldIndex = swarmWorld.exploredGraph.Nodes(curNode).trueGraphIndex;
prevNodeTrueWorldIndex = swarmWorld.exploredGraph.Nodes(prevNode).trueGraphIndex;

% compute heading of node
heading = atan2( trueWorld.nodeY(curNodeTrueWorldIndex) - trueWorld.nodeY(prevNodeTrueWorldIndex) , ...
                 trueWorld.nodeX(curNodeTrueWorldIndex) - trueWorld.nodeX(prevNodeTrueWorldIndex) );

% add new state to target state space graph 
NodeProps = table(curNode,prevNode,heading,'VariableNames',{'curNode','prevNode','heading'});
swarmWorld.tracking.targetStateSpaceGraph = addnode(swarmWorld.tracking.targetStateSpaceGraph,NodeProps);
curState = numnodes(swarmWorld.tracking.targetStateSpaceGraph);

% add new row and col to state-transition matrix
swarmWorld.tracking.stateTransitionMat(curState,:) = zeros(1,curState-1); 
swarmWorld.tracking.stateTransitionMat(:,curState) = zeros(curState,1); 

% update maps from explored environment node index to target state space 
swarmWorld.tracking.expEnvMap2CurNodeState{curNode} = [swarmWorld.tracking.expEnvMap2CurNodeState{curNode} curState];
swarmWorld.tracking.expEnvMap2PrevNodeState{prevNode} = [swarmWorld.tracking.expEnvMap2CurNodeState{prevNode} curState];

% add edges leading in to curState and update state transition matrix
% i.e., add edge from states that have curNode equal to curState's prevNode
for i = 1:1:length( swarmWorld.tracking.expEnvMap2CurNodeState{prevNode} )
    validPrevState = swarmWorld.tracking.expEnvMap2CurNodeState{prevNode}(i); 
    swarmWorld.tracking.targetStateSpaceGraph = addedge( swarmWorld.tracking.targetStateSpaceGraph, validPrevState, curState );
    swarmWorld.tracking.stateTransitionMat(curState, validPrevState) = stateTransitionProb(curState, validPrevState);
end

% add edges leading out of curState and update state transition matrix
% i.e., add edge from states that have prevNode equal to curState's curNode
for i = 1:1:length( swarmWorld.tracking.expEnvMap2PrevNodeState{curNode} )
    validNextState = swarmWorld.tracking.expEnvMap2PrevNodeState{curNode}(i); 
    swarmWorld.tracking.targetStateSpaceGraph = addedge( swarmWorld.tracking.targetStateSpaceGraph, curState, validNextState );
    swarmWorld.tracking.stateTransitionMat(curState, validPrevState) = stateTransitionProb();
end




end