function [signals, V, numViews] = simulateTargetSensorNoisy( swarmState, swarmModel, swarmWorld, trueWorld, targetState, targetModel )

% get targets xy
for i = 1:1:targetModel.M
    if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
        curNode = targetState.x(4*i-3,1);
    elseif( strcmp(targetModel.type, 'constantSpeedRandomWalk') || strcmp(targetModel.type,'constantSpeedRandomWalkGenerative') )
        curNode = targetState.x(2*i-1,1);
    end
    targNodes(i) = curNode; % on true graph
end

signals = [];
V = [];
numViews = swarmWorld.numViews;

% iterate over each agent
for i = 1:1:swarmModel.N
    
    % get agent xy position
    agent = [swarmState.x(4*i-3); swarmState.x(4*i-2)];
    
    % find the nodes in view for agent i
    if ( swarmModel.LRDTOnTheFlyFlag )
        Vagent = findNodesInViewExploredGraph(swarmWorld.exploredGraph, trueWorld.G_env, agent(1), agent(2), swarmModel.Rsense);
        if ( ~isempty(Vagent) ) 
            V_env = swarmWorld.exploredGraph.Nodes.trueGraphIndex(Vagent);
        else
            V_env = [];
        end
    else
        Vagent = findNodesInView(trueWorld.G_env, agent(1), agent(2), swarmModel.Rsense);
        V_env = Vagent;
    end
    
    
    signalsAgent = [];
    % for each node in view generate a measurement
    for j = 1:1:length(V_env)
        if ( any(V_env(j) == targNodes) )
            % disp('Target in view, generating signal');
            % if there is a target with this node ID then produce signal
            signalsAgent(j,1) = randn() + swarmModel.m;
            numViews = numViews + 1;
        else
            %otherwise noise           
            % disp('noise')
            signalsAgent(j,1) = randn();
        end
    end
    
    V = [ V; Vagent];
    signals = [signals; signalsAgent];
    
end

    if (length(V) ~= length(signals) )
       error('Error in V and signals'); 
    end
