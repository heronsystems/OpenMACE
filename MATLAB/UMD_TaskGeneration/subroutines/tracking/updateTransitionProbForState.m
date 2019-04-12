function Q = updateTransitionProbForState(si, Q, targetStateSpaceGraph, probStopping, m, d)
% get set of states reachable from state si
[~,reachableStates] = outedges(targetStateSpaceGraph, si);
Nrs = length(reachableStates);
% identify the reachable state that is at rest (it must exist)
sj_atRest_ind = [];
for j = 1:1:Nrs
    % next state
    sj = reachableStates(j);
    % check if the j-th node is a 'rest state'
    if ( targetStateSpaceGraph.Nodes.curNode(sj) == targetStateSpaceGraph.Nodes.prevNode(sj) )
        sj_atRest_ind = j;
    end
end
if ( isempty(sj_atRest_ind) )
    error('stateTransitionMatrix: rest state not found');
end

% assign the remaining probability to the remaining reachable states
if (Nrs == 1)
    Q(si, reachableStates(sj_atRest_ind) ) = 1; % special case with only one node
elseif ( Nrs > 1 )
    % populate the entry probability for coming to/staying at rest
    Q(si, reachableStates(sj_atRest_ind) ) = probStopping;
    % remove the rest state entry from the reachable set
    reachableStates( sj_atRest_ind ) = [];
    % check if the i-th state is itself a 'rest state'
    if ( targetStateSpaceGraph.Nodes.curNode(si) == targetStateSpaceGraph.Nodes.prevNode(si) )
        si_atRest = 1;
    else
        si_atRest = 0;
    end
    % if target state si is already at rest then the target moves to
    % another state with uniform probability (no heading/inertia)
    % considerations
    if ( si_atRest )
        weight = 1/(Nrs-1);
        % uniform probability if starting from rest
        Q(si,reachableStates) = (1-probStopping)*weight;
    else
        weights = zeros(1,Nrs-1);
        for j = 1:1:(Nrs-1)
            si_heading = targetStateSpaceGraph.Nodes.heading(si);
            sj_heading = targetStateSpaceGraph.Nodes.heading( reachableStates(j) );
            headingDiff = angularDist(si_heading, sj_heading);
            weights(j) = relWeightFun(headingDiff,m,d);
        end
        % normalize weights
        weights = weights / sum(weights);
        
        Q(si,reachableStates) = (1-probStopping)*weights;
    end
end

end