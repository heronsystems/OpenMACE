function swarmWorld = updateLikelihood(swarmWorld, swarmState, swarmModel, trueWorld, targetState, targetModel)

if ( ~isempty(swarmWorld.log_likelihood) )
    
% figure(1);
% subplot(4,1,1);
% plot(swarmWorld.log_likelihood,'mo-');
% hold on;
% ylabel('log LR');
% xlabel('Target State Index');
% subplot(4,1,2);
% plot(exp(swarmWorld.log_likelihood),'mo-');
% hold on;
% ylabel('LR');
% xlabel('Target State Index');

% 1. Motion Update
% -------------------------------------------------------------------------

% motion update
if ( swarmModel.LRDTOnTheFlyFlag  )
     ns = length(swarmWorld.log_likelihood);

    % likelihood ratio 
    L = exp(swarmWorld.log_likelihood);
    sumL = sum(L);
    % unpack likelihood intro a probability for each state
    ps_prior = L/(1+sumL);
    pn_prior = 1/(1+sumL);
    ps = swarmWorld.Q'*ps_prior + ones(ns,1)*swarmModel.q_s_n*pn_prior/ns;
    pn = ones(1,ns)*swarmModel.q_n_s*ps_prior + swarmModel.q_n_n*pn_prior;
    
    % error check
    if ( abs(sum(ps) + pn - 1) > 0.001 )
        error('probability is significantly off');
    end
    likelihood = ps/pn;
    
    % Previous approach
    %likelihood = swarmWorld.Q'*(exp(swarmWorld.log_likelihood));
    %likelihood = likelihood ./ (ns*swarmModel.q_s_n*likelihood + 1);
else
    likelihood = trueWorld.Q'*(exp(swarmWorld.log_likelihood));
end

% convert back to log LR
swarmWorld.log_likelihood = log(likelihood); % 

% 2. Measurement Update
% -------------------------------------------------------------------------

% % debug
%fprintf('Cuml LR before msmt update : %3.3f\n', sum(exp(swarmWorld.log_likelihood)) );
% figure(1);
% subplot(4,1,1);
% plot(swarmWorld.log_likelihood,'ko-');
% hold on;
% subplot(4,1,2);
% plot(exp(swarmWorld.log_likelihood),'ko-');
% hold on;


switch swarmModel.sensorType
    case 'discrete_per_cell'
        
        % a signal is generated for each node in each agent's field of view
        [signals, V, swarmWorld.numViews] = simulateTargetSensorCellWise( swarmState, swarmModel, swarmWorld, trueWorld, targetState, targetModel );
        
        % if all agents are in void space then no update is required
        if ( ~isempty(signals) )
            logMsmtLikelihood = swarmModel.mZ*(signals - swarmModel.mZ/2); % convert signals to log-likelihood ratio
            
            % error check
            if ( length(signals) ~= length(V) )
                error('Incorrect lengths of signals and nodes v');
            end
            % apply likelihood update to each node
            for j = 1:1:length(V)
                % find the states corresponding to the nodes in view
                if ( swarmModel.LRDTOnTheFlyFlag )
                    S = findTargStatesInView(swarmWorld.Mc, V(j));
                else
                    S = findTargStatesInView(trueWorld.Mc, V(j));
                end
                for k = 1:1:length(S)
                    swarmWorld.log_likelihood(S(k)) =  swarmWorld.log_likelihood(S(k)) + logMsmtLikelihood(j);
                end
            end
        end
        if ( isempty(V) )
            disp('No Nodes in View');
        end
        swarmWorld.nodesInView = V;
    case 'continuous_per_fov'
        % here one signal is generated per agent
        [signals] = simulateTargetSensor( swarmState, swarmModel, trueWorld, targetState, targetModel );
        logMsmtLikelihood = swarmModel.m*(signals - swarmModel.m/2); % convert signals to log-likelihood ratio
        for i = 1:1:swarmModel.N
            % extract agent position
            agent = [swarmState.x(4*i-3) swarmState.x(4*i-2)];
            % find the nodes in view for agent i
            if ( swarmModel.LRDTOnTheFlyFlag )
                V = findNodesInViewExploredGraph(swarmWorld.exploredGraph, trueWorld.G_env, agent(1), agent(2), swarmModel.Rsense);
                S = findTargStatesInView(swarmWorld.Mc, V);
            else
                V = findNodesInView(trueWorld.G_env, agent(1), agent(2), swarmModel.Rsense);
                % find the corresponding states
                S = findTargStatesInView(trueWorld.Mc, V);
            end
            
            % apply likelihood update to each node
            for j = 1:1:length(S)
                swarmWorld.log_likelihood(S(j)) =  swarmWorld.log_likelihood(S(j)) + logMsmtLikelihood(j);
            end
            
        end
        
        
end

% %debug
% subplot(4,1,1);
% plot(swarmWorld.log_likelihood,'ro-');
% hold on;
% legend('Prior','Motion Update','Measurement Update');
% subplot(4,1,2);
% plot(exp(swarmWorld.log_likelihood),'ro-');
% legend('Prior','Motion Update','Measurement Update');
% subplot(4,1,3);
% plot(signals,'ro-');
% hold on;
% plot(ones(size(signals))*swarmModel.mZ,'k-');
% ylabel('Signal')
% xlabel('Index');
% 
% subplot(4,1,4);
% plot(logMsmtLikelihood,'bo-');
% hold on;
% ylabel('Log-likelihood from msmt')
% xlabel('Node in View Index');




swarmWorld.signals = signals;

%fprintf('Cuml LR after msmt update : %3.3f\n', sum(exp(swarmWorld.log_likelihood)) );
% 4. Normalize and compute cumulative likelihood ratio
% -------------------------------------------------------------------------

% recover bayesian probabilities (unscaled)
likelihood = exp(swarmWorld.log_likelihood);
likelihood_sum = sum( likelihood );
swarmWorld.tss_probPresent = likelihood / (1 + likelihood_sum);
swarmWorld.tss_probAbsent = 1 / (1 + likelihood_sum);
totalProb_sum = sum(swarmWorld.tss_probPresent)  +  swarmWorld.tss_probAbsent;

% normalize
swarmWorld.tss_probPresent = swarmWorld.tss_probPresent / totalProb_sum;
swarmWorld.tss_probAbsent = swarmWorld.tss_probAbsent / totalProb_sum;

swarmWorld.env_probPresent = projectLikelihood( swarmWorld.tss_probPresent , swarmWorld.Mc );
swarmWorld.log_likelihood = log( swarmWorld.tss_probPresent / swarmWorld.tss_probAbsent );

% 3. Project likelihood
% -------------------------------------------------------------------------
% this projection operator sums along the states whose current node is the
% one of interest

if ( swarmModel.LRDTOnTheFlyFlag )    
    swarmWorld.log_likelihood_env = log(projectLikelihood( exp(swarmWorld.log_likelihood) , swarmWorld.Mc ));
else
    swarmWorld.log_likelihood_env = log(projectLikelihood( exp(trueWorld.log_likelihood) , trueWorld.Mc ));
end


% check
if ( any( isnan(swarmWorld.log_likelihood) ) )
    disp('Error: NaN in log_likelihood');
end

% cumulative likelihood ratio
if ( swarmModel.LRDTOnTheFlyFlag )
    swarmWorld.cumlLR = sum(exp(swarmWorld.log_likelihood));
else
    swarmWorld.cumlLR = sum(exp(trueWorld.log_likelihood));
end

% Call detections
% -------------------------------------------------------------------------
if ( swarmWorld.cumlLR >= swarmModel.cumlLRthresh && swarmWorld.targetDetectedFlag==0 )
    swarmWorld.targetDetectedFlag = 1;
    disp('Target Detected!')
    fprintf('Target Detected! cumlLR = %3.1f >= %3.1f \n', swarmWorld.cumlLR , swarmModel.cumlLRthresh);
    
    swarmWorld.timeAtDetection = swarmState.t;
    % check if target detected is accurate or not
    [maxVal, maxInd] = max(swarmWorld.env_probPresent); %log_likelihood_env);
    switch swarmModel.mappingSensorType
        case 'perfect'
            trueGraphIndexMaxProb = swarmWorld.exploredGraph.Nodes.trueGraphIndex(maxInd);
            predictedTargXY = [trueWorld.nodeX(trueGraphIndexMaxProb) trueWorld.nodeY(trueGraphIndexMaxProb)];
        case 'noisy'
            bx = swarmWorld.exploredGraph.Nodes.bx(maxInd);
            by = swarmWorld.exploredGraph.Nodes.by(maxInd);
            predictedTargXY = [trueWorld.xcp(bx) trueWorld.ycp(by)];
    end
    
    
    
    % get targets xy
    for i = 1:1:targetModel.M
        if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
            curNode = targetState.x(4*i-3,1);
        elseif( strcmp(targetModel.type, 'constantSpeedRandomWalk') || strcmp(targetModel.type, 'constantSpeedRandomWalkGenerative') )
            curNode = targetState.x(2*i-1,1);
        end
        targNodes(i) = curNode;
    end
    targetsXY = [ trueWorld.nodeX(targNodes) , trueWorld.nodeY(targNodes) ];
    % compute distance to each target
    for j = 1:1:targetModel.M
        distToTargs(j) = norm( predictedTargXY - targetsXY(j,:) );
    end
    
    % check if any target is in view
    targsInView = find( distToTargs < swarmModel.Rsense );
    
    % if there is a target in view:
    if ( ~isempty(targsInView) )
        swarmWorld.targetDetectionValid = 1;
        
        disp('Detection is valid');
    else  % if there is no target in view:
        swarmWorld.targetDetectionValid = 0;
        disp('Distance to targets');
        distToTargs
        disp('Target Locations')
        targetsXY
        disp('Predicted XY')
        predictedTargXY
        disp('Detection is invalid!');
    end
end

% update U, V 
% -------------------------------------------------------------------------

% figure;
% subplot(2,2,1)
% imagesc(swarmWorld.V); caxis([0 1])
% subplot(2,2,2)
% imagesc(swarmWorld.U); caxis([0 1])
% subplot(2,2,3)
% imagesc(swarmWorld.O); caxis([0 1])
% subplot(2,2,4)
% imagesc(swarmWorld.O./swarmWorld.U);
% colorbar;
% title('Likelihood prior')

    sumL = sum(exp(swarmWorld.log_likelihood_env));
    %nullStateProb = swarmWorld.U(by,bx) = 1/(1 + sumL); % prob no target
for i = 1:1:length(swarmWorld.log_likelihood_env)
    bx = swarmWorld.exploredGraph.Nodes.bx(i);
    by = swarmWorld.exploredGraph.Nodes.by(i);
    L = exp(swarmWorld.log_likelihood_env(i));
    swarmWorld.V(by,bx) = 0;    
    
    %swarmWorld.O(by,bx) = L/(1 + sumL); % prob occupied
    swarmWorld.O(by,bx) = swarmWorld.env_probPresent(i);
    swarmWorld.U(by,bx) = (1 - swarmWorld.O(by,bx));
end

% figure;
% subplot(2,2,1)
% imagesc(swarmWorld.V); caxis([0 1])
% subplot(2,2,2)
% imagesc(swarmWorld.U); caxis([0 1])
% subplot(2,2,3)
% imagesc(swarmWorld.O); caxis([0 1])
% subplot(2,2,4)
% imagesc(swarmWorld.O./swarmWorld.U);
% colorbar;
% title('Likelihood update')
% 
% 1;
% % initialize
% numRows = size(cellStateMat,1); % rows
% numCols = size(cellStateMat,2); % cols
%
% % iterate through each grid cell
% for i = 1:1:numRows
%     for j = 1:1:numCols
%         switch cellDetMat(i,j)
%             case 1 % if cell is explored and is a node then assign prior based on likelihood
%                 P(i,j) = 0;
%                 % map bin i,j to node ID
%                 trueGraphNodeID = bin2NodeID(i,j);
%                 % find explored graph node ID
%                 exploredGraphNodeID = find( exploredGraph.Nodes.trueGraphIndex == trueGraphNodeID );
%                 % find the node in the
%
%     end
% end

end


end
