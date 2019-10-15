function [swarmWorld] = buildTargetStateSpaceGraphOnTheFly( swarmWorld, swarmModel, trueWorld, targetModel, idNew )

% add new target states
N = neighbors(swarmWorld.exploredGraph,idNew);
numStatesAdded = 0;

% add rest-state
switch swarmModel.mappingSensorType
    case 'perfect'
        [swarmWorld.targetStateSpaceGraph,swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp] = addNodeToTargetStateSpace(idNew, idNew, swarmWorld.targetStateSpaceGraph, swarmWorld.exploredGraph, trueWorld.G_env, swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp);
    case 'noisy'
        [swarmWorld.targetStateSpaceGraph,swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp] = addNodeToTargetStateSpaceNoisyMap(idNew, idNew, swarmWorld.targetStateSpaceGraph, swarmWorld.exploredGraph, swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp, trueWorld.xcp, trueWorld.ycp);
end
numStatesAdded = numStatesAdded + 1;

% add outgoing/incoming states
for i = 1:1:length(N)
    switch swarmModel.mappingSensorType
        case 'perfect'
            [swarmWorld.targetStateSpaceGraph,swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp] = addNodeToTargetStateSpace(idNew,N(i), swarmWorld.targetStateSpaceGraph, swarmWorld.exploredGraph, trueWorld.G_env, swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp);
            [swarmWorld.targetStateSpaceGraph,swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp] = addNodeToTargetStateSpace(N(i),idNew, swarmWorld.targetStateSpaceGraph, swarmWorld.exploredGraph, trueWorld.G_env, swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp);
        case 'noisy'
            [swarmWorld.targetStateSpaceGraph,swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp] = addNodeToTargetStateSpaceNoisyMap(idNew,N(i), swarmWorld.targetStateSpaceGraph, swarmWorld.exploredGraph, swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp, trueWorld.xcp, trueWorld.ycp);
            [swarmWorld.targetStateSpaceGraph,swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp] = addNodeToTargetStateSpaceNoisyMap(N(i),idNew, swarmWorld.targetStateSpaceGraph, swarmWorld.exploredGraph, swarmWorld.Mpc2s,swarmWorld.Mc,swarmWorld.Mp, trueWorld.xcp, trueWorld.ycp);
    end
    numStatesAdded = numStatesAdded + 2;
end

% add zero-log likelihood for all the new states
%swarmWorld.log_likelihood = [swarmWorld.log_likelihood; zero(numStatesAdded,1)];


% add nom likelihood to all new states
if ( isempty( swarmWorld.targetStateSpaceGraph.Nodes ) )
    Ns = numStatesAdded;
else
    Ns = numStatesAdded + length(swarmWorld.targetStateSpaceGraph.Nodes.prevNode);
end

% baseline prob
pTemp = -Inf;

% add zero-log likelihood for all the new states
swarmWorld.log_likelihood = [swarmWorld.log_likelihood; ones(numStatesAdded,1)*pTemp];

% keep a running list of nodes whose edges have been modified, the swarmWorld.Q
% entries of these will need to be modified
modifiedNodes = [];

% define case 1a edges
v = idNew; % for convenience
Nv = neighbors(swarmWorld.exploredGraph,v);
if ( ~isempty(Nv) )
    for y = Nv'
        Ny = neighbors(swarmWorld.exploredGraph,y);
        if ( ~isempty(Ny) )
            for z = Ny'
                n1 = swarmWorld.Mpc2s(v,y);
                n2 = swarmWorld.Mpc2s(y,z);
                swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
                modifiedNodes = [modifiedNodes; n1 n2];
            end
        end
        % special case z = y
        z = y;
        n1 = swarmWorld.Mpc2s(v,y);
        n2 = swarmWorld.Mpc2s(y,z);
        swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
        modifiedNodes = [modifiedNodes; n1 n2];
    end
end

% define case 1b edges
if ( ~isempty(Nv) )
    for z = Nv'
        n1 = swarmWorld.Mpc2s(v,v);
        n2 = swarmWorld.Mpc2s(v,z);
        swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
        modifiedNodes = [modifiedNodes; n1 n2];
    end
end
% 1b special case z = v;
n1 = swarmWorld.Mpc2s(v,v);
n2 = swarmWorld.Mpc2s(v,v);
swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
modifiedNodes = [modifiedNodes; n1 n2];

% define case 2a edges
if ( ~isempty(Nv) )
    for x = Nv'
        Nx = neighbors(swarmWorld.exploredGraph,x);
        if ( ~isempty(Nx) )
            for y = Nx'
                if ( any(Nv==y) && y ~= v  )
                    n1 = swarmWorld.Mpc2s(x,y);
                    n2 = swarmWorld.Mpc2s(y,v);
                    swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
                    modifiedNodes = [modifiedNodes; n1 n2];
                end
            end
        end
    end
end

% define case 2b edges
if ( ~isempty(Nv) )
    for x = Nv'
        n1 = swarmWorld.Mpc2s(x,x);
        n2 = swarmWorld.Mpc2s(x,v);
        swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
        modifiedNodes = [modifiedNodes; n1 n2];
    end
end

% define case 2c edges
if ( ~isempty(Nv) )
    for x = Nv'
        for z = Nv'
            n1 = swarmWorld.Mpc2s(x,v);
            n2 = swarmWorld.Mpc2s(v,z);
            swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
            modifiedNodes = [modifiedNodes; n1 n2];
        end
    end
end

% define case 2d edges
z = v;
if ( ~isempty(Nv) )
    for x = Nv'
        n1 = swarmWorld.Mpc2s(x,v);
        n2 = swarmWorld.Mpc2s(v,z);
        swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
        modifiedNodes = [modifiedNodes; n1 n2];
    end
end

% define case 3a edges
if ( ~isempty(Nv) )
    for y = Nv'
        Ny = neighbors(swarmWorld.exploredGraph,y);
        if ( ~isempty(Ny) )
            for x = Ny'
                if ( ~any(x==Nv) && x ~= v )
                    n1 = swarmWorld.Mpc2s(x,y);
                    n2 = swarmWorld.Mpc2s(y,v);
                    swarmWorld.targetStateSpaceGraph = addedge(swarmWorld.targetStateSpaceGraph, n1, n2);
                    modifiedNodes = [modifiedNodes; n1 n2];
                end
            end
        end
    end
end

% now we update swarmWorld.Q
modifiedNodes = unique(modifiedNodes); % remove repeats
Ns = numnodes(swarmWorld.targetStateSpaceGraph);
swarmWorld.Q = swarmWorld.Q .* 1/(1-swarmModel.q_n_s); %  normalize temporarily
for i = 1:1:length(modifiedNodes)
    si = modifiedNodes(i);
    swarmWorld.Q = updateTransitionProbForState(si, swarmWorld.Q, swarmWorld.targetStateSpaceGraph, targetModel.probStopping, targetModel.m, targetModel.d);
end

% check that swarmWorld.Q is row stochastic
tol = 0.0001;
for i = 1:1:size(swarmWorld.Q,1)
    if ( abs(sum(swarmWorld.Q(i,:)) - 1)>tol)
        fprintf('Row %d sums to %3.3f \n',i, sum(swarmWorld.Q(i,:)));
        error('swarmWorld.Q is not row stochastic (temporarily)');
    end
end

% each state transitions into the null state with probability:
% swarmModel.q_n_s
% so each row must now sum to (1-swarmModel.q_n_s) this
% is achieved by scaling
swarmWorld.Q = swarmWorld.Q .* (1-swarmModel.q_n_s);

end



