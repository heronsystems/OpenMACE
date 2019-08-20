function swarmWorld = updateLikelihood(swarmWorld, swarmState, swarmModel, trueWorld, targetState, targetModel)

if ( ~isempty(swarmWorld.log_likelihood) )
    
    % 1. Motion Update
    % -------------------------------------------------------------------------
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
    else
        likelihood = trueWorld.Q'*(exp(swarmWorld.log_likelihood));
    end
    
    % convert back to log LR
    swarmWorld.log_likelihood = log(likelihood); %
    
    % 2. Measurement Update
    % -------------------------------------------------------------------------
    
    % a signal is generated for each node in each agent's field of view
    %[signals, V, swarmWorld.numViews] = simulateTargetSensorCellWise( swarmState, swarmModel, swarmWorld, trueWorld, targetState, targetModel );
    
    % Note : swarmWorld.cellsInView is of format [bx by] in each row
    V = []; % nodes in view corresponding to target signals
    signals = [];
    for i = 1:1:length(swarmWorld.cellsInView)
        nodeID = swarmWorld.bin2NodeIDexplored( swarmWorld.cellsInView(i,1) , swarmWorld.cellsInView(i,2) );
        if ( nodeID ~= 0 )
            signals = [signals swarmModel.zval(swarmWorld.targSignals(i))]; % index values
            V = [V nodeID];
        end
    end
    
    
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
    
    swarmWorld.log_likelihood_env = log(projectLikelihood( exp(swarmWorld.log_likelihood) , swarmWorld.Mc ));
    
    % check
    if ( any( isnan(swarmWorld.log_likelihood) ) )
        disp('Error: NaN in log_likelihood');
    end
    
    % cumulative likelihood ratio
    swarmWorld.cumlLR = sum(exp(swarmWorld.log_likelihood));
    
    % Call detections
    % -------------------------------------------------------------------------
    if ( swarmWorld.cumlLR >= swarmModel.cumlLRthresh && swarmWorld.targetDetectedFlag==0 )
        swarmWorld.targetDetectedFlag = 1;
        disp('Target Detected!')
        fprintf('\n\n\n\nTarget Detected! cumlLR = %3.1f >= %3.1f \n\n\n\n', swarmWorld.cumlLR , swarmModel.cumlLRthresh);
        
        swarmWorld.timeAtDetection = swarmState.t;
        % check if target detected is accurate or not
        [maxVal, maxInd] = max(swarmWorld.env_probPresent); %log_likelihood_env);
        bx = swarmWorld.exploredGraph.Nodes.bx(maxInd);
        by = swarmWorld.exploredGraph.Nodes.by(maxInd);
        predictedTargXY = [trueWorld.xcp(bx) trueWorld.ycp(by)];
        
        % get targets xy
        for i = 1:1:targetModel.M
            curNode = targetState.x(2*i-1,1);
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
    sumL = sum(exp(swarmWorld.log_likelihood_env));
    for i = 1:1:length(swarmWorld.log_likelihood_env)
        bx = swarmWorld.exploredGraph.Nodes.bx(i);
        by = swarmWorld.exploredGraph.Nodes.by(i);
        L = exp(swarmWorld.log_likelihood_env(i));
        swarmWorld.V(by,bx) = 0;
        swarmWorld.O(by,bx) = swarmWorld.env_probPresent(i);
        swarmWorld.U(by,bx) = (1 - swarmWorld.O(by,bx));
    end
    
end


end
