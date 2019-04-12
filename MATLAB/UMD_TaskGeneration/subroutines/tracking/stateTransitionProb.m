function prob = stateTransitionProb(state, swarmWorld)

% determine if it is feasible to go from startState to endState
if ( swarmWorld.targetStateSpaceGraph.Nodes(endState).curNode ==  swarmWorld.targetStateSpaceGraph.Nodes(endState).prevNode )
    % check if toState is a "rest" state
    if ( swarmWorld.targetStateSpaceGraph.Nodes(startState).prevNode == swarmWorld.targetStateSpaceGraph.Nodes(startState).curNode )
        % check if this is a special case where the state is completely
        % disconnected from all others
        if ( (startState == endState) && length(swarmWorld.targetStateSpaceGraph.Nodes(startState).prevNode) == 1 )
            prob = 1; % there is no where else to go so probability is 1
        else
            prob = ps; % this is the probability of stopping
        end
        % check if fromState is a "rest" state
    elseif ( swarmWorld.targetStateSpaceGraph.Nodes(endState).prevNode == swarmWorld.targetStateSpaceGraph.Nodes(endState).curNode )
        % compute how many possible moves there are
        fromStateCurNode = swarmWorld.targetStateSpaceGraph.Nodes(endState).curNode;
        numMoves = length( swarmWorld.tracking.expEnvMap2PrevNodeState{fromStateCurNode} )-1; % subtract the move of staying stationary
        % if starting from rest we take the inertia to be zero
        prob = (1-ps)*1/numMoves;
        % implies both states are in motion
    else
        % use the inertia and update for all of the states
        
    end
else
    prob = 0;
    warning('stateTransitionProb: Evaluating probability of joining two dis-conected states');
end


end
