function [signals, V, numViews] = simulateTargetSensorCellWise( swarmState, swarmModel, swarmWorld, trueWorld, targetState, targetModel )

% get targets xy
for i = 1:1:targetModel.M
    if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
        curNode = targetState.x(4*i-3,1);
    elseif( strcmp(targetModel.type, 'constantSpeedRandomWalk') || strcmp(targetModel.type,'constantSpeedRandomWalkGenerative') )
        curNode = targetState.x(2*i-1,1);
    end
    targNodes(i) = curNode; % on true graph
    targXY(i,1) = trueWorld.G_env.Nodes.x( curNode );
    targXY(i,2) = trueWorld.G_env.Nodes.y( curNode );
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
        switch swarmModel.mappingSensorType
            case 'perfect'
                Vagent = findNodesInViewExploredGraph(swarmWorld.exploredGraph, trueWorld.G_env, agent(1), agent(2), swarmModel.Rsense);
                if ( ~isempty(Vagent) )
                    V_env = swarmWorld.exploredGraph.Nodes.trueGraphIndex(Vagent);
                else
                    V_env = [];
                end
            case 'noisy'
                Vagent = findNodesInViewExploredGraphNoisyMap(swarmWorld.exploredGraph, agent(1), agent(2), swarmModel.Rsense, trueWorld.xcp, trueWorld.ycp);
%                 %debug
%                 fprintf('Agent %d sees %d nodes\n',i,length(Vagent));
%                 figure(100);
%                 plot(agent(1),agent(2),'ko','MarkerSize',3);
%                 hold on;
%                 for j = 1:1:length(Vagent)
%                     xn = trueWorld.xcp( swarmWorld.exploredGraph.Nodes.bx( Vagent(j) ) );
%                     yn = trueWorld.ycp( swarmWorld.exploredGraph.Nodes.by( Vagent(j) ) );
%                    plot(xn,yn,'x');
%                 end
%                 hold on;
%                 plot(trueWorld.xpoly,trueWorld.ypoly);
                
        end
    else
        Vagent = findNodesInView(trueWorld.G_env, agent(1), agent(2), swarmModel.Rsense);
        V_env = Vagent;
    end
    
    
    
    signalsAgent = [];
    % for each node in view generate a measurement
    switch swarmModel.mappingSensorType
        case 'perfect'
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
        case 'noisy'
            Vagent = findNodesInViewExploredGraphNoisyMap(swarmWorld.exploredGraph, agent(1), agent(2), swarmModel.Rsense, trueWorld.xcp, trueWorld.ycp);
            for j = 1:1:length(Vagent)
                % target sensor
                bx = swarmWorld.exploredGraph.Nodes.bx(Vagent(j));
                by = swarmWorld.exploredGraph.Nodes.by(Vagent(j));
                controlPt = [trueWorld.xcp(bx) trueWorld.ycp(by)];
                for k = 1:1:targetModel.M
                    if ( norm(controlPt - targXY(k,:)) == 0 )
                        %signalsAgent(j,1) = randn() + swarmModel.m; %quantizedSensor(mZ, nZ, 1);
                        signalsAgent(j,1) = swarmModel.zval( quantizedSensor(swarmModel.mZ, swarmModel.nZ, 1) );
                        numViews = numViews + 1;
                        disp('Target in view');
                        fprintf('total signal = %3.3f, random = %3.3f, bias = %3.3f \n', signalsAgent(j,1), signalsAgent(j,1) - swarmModel.mZ, swarmModel.mZ);
                    else
                        %signalsAgent(j,1) = randn(); %quantizedSensor(mZ, nZ, 0);
                         signalsAgent(j,1) = swarmModel.zval( quantizedSensor(swarmModel.mZ, swarmModel.nZ, 0) );
                    end
                end
            end
            
    end
    
    
    
    V = [ V; Vagent];
    signals = [signals; signalsAgent];
    
end

if (length(V) ~= length(signals) )
    error('Error in V and signals');
end
