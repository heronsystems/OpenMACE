function [swarmWorldHist, swarmStateHist, targetStateHist] = simulateMatlab(targetState, targetModel, swarmState, swarmModel, swarmWorld, trueWorld, runParams)
% histories are used for performance analysis and plotting
s = 1; % number of samples
swarmWorldHist{1} = swarmWorld;
swarmStateHist{1} = swarmState;
targetStateHist{1} = targetState;

t(1) = 0;        % simulate targets
targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams, 1);

for k = 2:1:runParams.Nsim
%     tCycle = tic;
    if ( swarmWorld.targetDetectedFlag && swarmModel.terminateSimOnDetect )
        break;
        disp('Terminating simulation (target detected)');
    end
    switch swarmModel.communicationTopology
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'centralized'
            if ( mod( swarmState.k , floor(swarmModel.Tsamp/runParams.dt) ) == 0 )
                tSample = tic;
                % current time
                swarmWorld.time = k*runParams.dt;
                % simulate measurements/update the swarmWorld
                swarmWorld = updateSwarmWorld(swarmWorld, swarmState, swarmModel, trueWorld, targetModel, targetState);
                
                
                % update likelihood surface
                swarmWorld = updateLikelihood(swarmWorld, swarmState, swarmModel, trueWorld, targetState, targetModel);
                %                 % mutual information surface
                %                 [swarmWorld.mutualInfoSurface, swarmWorld.priorP, swarmWorld.priorQ, swarmWorld.priorR, swarmWorld.predictedNodeProbMat, swarmWorld.nodeDensityEstimate, swarmWorld.entropyMat, swarmWorld.totalEntropy] = mutualInformationSurface( ...
                %                     swarmModel.maxUnexploredPrior, swarmModel.m, swarmModel.sensorDiscretizationLevels, ...
                %                     swarmModel.mapping.krigingSigma, trueWorld.xx, trueWorld.yy,  trueWorld.bin2NodeID, ...
                %                     swarmWorld.exploredGraph, swarmWorld.log_likelihood_env,  swarmWorld.cellStateMat , swarmModel.probAbsentPrior );
                %
                             
                
                
                %                 % mutal Info
                %                 tic;
                %                 %swarmWorld.mutualInfoSurface
                %                 I_C_GZ_old = mutualInformationMappingTargetOld(swarmWorld.V, swarmWorld.U, swarmWorld.O, ...
                %                     swarmModel.sensorDiscretizationLevels, swarmModel.nG, swarmModel.z_VU, swarmModel.z_O, swarmModel.g_V, swarmModel.g_UO);
                %                 disp('mutualInfo');toc;
                %
                [swarmWorld.entropyMat, swarmWorld.mutualInfoSurface, swarmWorld.totalEntropy] = mutualInformationMappingTarget(swarmWorld.V, swarmWorld.U, swarmWorld.O, swarmModel.z_VU, swarmModel.z_O, swarmModel.g_V, swarmModel.g_UO);

                
                %                 figure;
                %                 subplot(2,1,1);
                %                 imagesc(I_C_GZ_old);
                %                 subplot(2,1,2);
                %                 imagesc(I_C_GZ_new);
                % entropy
                %[swarmWorld.entropyMat, swarmWorld.totalEntropy] = entropyMatrix(swarmWorld.V, swarmWorld.U, swarmWorld.O);

                % simulate targets
                integerTime = 1+swarmState.k/floor(swarmModel.Tsamp/runParams.dt);
                targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams, integerTime);
                
            end
            if ( mod( swarmState.k , swarmModel.samplesPerTask/runParams.dt ) == 0 )
                % generate/allocate tasks at new sampling times
                % task generation
                [tasks, swarmWorld] = taskGeneration(swarmWorld, swarmModel, trueWorld);
                % task allocation
                [swarmState, swarmWorld] = taskAllocation(tasks, swarmState, swarmModel, swarmWorld, trueWorld, runParams);
            end
            
            [swarmState] = taskManagement(swarmState, swarmModel, swarmWorld);
            
            % generate/allocate tasks at new sampling times
            if ( mod( swarmState.k , floor(swarmModel.Tsamp/runParams.dt) ) == 0 )
                % save the swarm world for later use
                swarmWorldHist{s} = swarmWorld;
                swarmStateHist{s} = swarmState;
                targetStateHist{s} = targetState;
                s = s + 1;
            end
            
            % simulate searchers
            xdot = swarmDynamics(swarmState, swarmModel);
            swarmState.x = swarmState.x + xdot' * runParams.dt; % euler's method
            % update time
            t(k) = t(k-1) + runParams.dt;
            
            % display
            if ( mod(k,floor(runParams.Nsim/50)) == 0 || k == 2 )
                fprintf('**** Run Percent : %3.0f **** \n', k/runParams.Nsim*100)
            end
            
            swarmState.t = t(k);
            swarmState.k = k;
            targetState.k = k;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'allToAll'
            for i = 1:swarmModel.N
                if ( mod( swarmState{i}.k , floor(swarmModel.Tsamp/runParams.dt) ) == 0 )
                    % note: using swarmState{1}.k  means all the agents are syncronized
                    % simulate measurements/update the swarmWorld
                    swarmWorld{i} = updateSwarmWorld(swarmWorld{i}, swarmState{i}, swarmModel, trueWorld, targetModel);
                    swarmWorld{i}.time = k;
                    % update likelihood surface
                    %             swarmWorld = updateLikelihood(swarmWorld, swarmState, swarmModel, trueWorld, targetState, targetModel);
                    % simulate targets
                    %             targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams);
                end
            end
            
            % messageExchange ()
            if ( mod( swarmState{1}.k , floor(swarmModel.Tsamp/runParams.dt) ) == 0 )
                swarmWorld = syncSituationalAwareness(swarmWorld,swarmState,swarmModel,trueWorld.G_env);
            end
            
            %                 % normal task allocation (greedy)
            
            %                 if ( mod( swarmState{1}.k , swarmModel.samplesPerTask ) == 0 )
            %                     for i = 1:swarmModel.N
            %                     	% note: using swarmState{1}.k  means all the agents are syncronized
            %             			% generate/allocate tasks at new sampling times
            %             			[tasks, swarmWorld{i}] = taskGeneration(swarmWorld{i}, swarmModel, trueWorld);
            %             			% task allocation
            %             			swarmState{i} = taskAllocation(tasks, swarmState{i}, swarmModel, swarmWorld{i}, trueWorld, runParams);
            %                 	end
            %                 end
            
            %               % task allocation verification for all-to-all
            %                 communication (Hungarian)
            
            if ( mod( swarmState{1}.k , swarmModel.samplesPerTask/runParams.dt ) == 0 )
                for i = 1:swarmModel.N
                    % note: using swarmState{1}.k  means all the agents are syncronized
                    % generate/allocate tasks at new sampling times
                    [tasks, swarmWorld{i}] = taskGeneration(swarmWorld{i}, swarmModel, trueWorld);
                end
                swarmState = taskAllocation_allToAll_verification(tasks, swarmState, swarmModel, swarmWorld, trueWorld, runParams);
            end
            
            % generate/allocate tasks at new sampling times
            if ( mod( swarmState{1}.k , floor(swarmModel.Tsamp/runParams.dt) ) == 0 )
                % save the swarm world for later use
                swarmWorldHist{s} = swarmWorld;
                swarmStateHist{s} = swarmState;
                targetStateHist{s} = targetState;
                s = s + 1;
            end
            % simulate searchers
            xdot = swarmDynamics(swarmState, swarmModel);
            for i = 1:swarmModel.N
                swarmState{i}.x = swarmState{i}.x + xdot{i}' * runParams.dt; % euler's method
            end
            % update time
            t(k) = t(k-1) + runParams.dt;
            
            % display
            if ( mod(k,floor(runParams.Nsim/20)) == 0 || k == 2 )
                fprintf('Run Percent : %3.0f \n', k/runParams.Nsim*100)
            end
            for i = 1:swarmModel.N
                swarmState{i}.t = t(k);
                swarmState{i}.k = k;
            end
            targetState.k = k;
    end
%     disp('Cycle Time');
%     toc(tCycle);
%     disp('-------')
end
end