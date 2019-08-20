for kk = 1:100

% clear; close all; clc;
% format compact;
updatePath;

[runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_cityblocks();

[swarmWorld, swarmState, targetState, ROS_MACE] = initializeRun(trueWorld, swarmModel, targetModel, runParams, ROS_MACE);
% histories are used for performance analysis and plotting
s = 1; % number of samples
targetStateHist{1} = targetState;

t(1) = 0;        % simulate targets
targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams,1);

visitedBoundary = [trueWorld.nodeX(targetState.x(1)) trueWorld.nodeX(targetState.x(1)) trueWorld.nodeY(targetState.x(1)) trueWorld.nodeY(targetState.x(1))];  % [xmin xmax ymin ymax]

for k = 2:1:runParams.Nsim
    if ( mod( swarmState.k , floor(swarmModel.Tsamp/runParams.dt) ) == 0 )
        % simulate targets
        targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams,1);
        
        visitedBoundary = [min(visitedBoundary(1),trueWorld.nodeX(targetState.x(1))) ...
            max(visitedBoundary(2),trueWorld.nodeX(targetState.x(1))) ...
            min(visitedBoundary(3),trueWorld.nodeY(targetState.x(1))) ...
            max(visitedBoundary(4),trueWorld.nodeY(targetState.x(1)))];
    end
    
    % generate/allocate tasks at new sampling times
    if ( mod( swarmState.k , floor(swarmModel.Tsamp/runParams.dt) ) == 0 )
        % save the swarm world for later use
        targetStateHist{s} = targetState;
        s = s + 1;
    end
    
    % update time
    t(k) = t(k-1) + runParams.dt;
    
    % display
    if ( mod(k,floor(runParams.Nsim/50)) == 0 || k == 2 )
        fprintf('**** Run Percent : %3.0f **** \n', k/runParams.Nsim*100)
    end
    
    swarmState.t = t(k);
    swarmState.k = k;
    targetState.k = k;
end

% maxVisitedRatio = (visitedBoundary(2)-visitedBoundary(1))*((visitedBoundary(4)-visitedBoundary(3)))/((trueWorld.maxX-trueWorld.minX)*(trueWorld.maxY-trueWorld.minY));

% matFileName = ['targetMotionData_' datestr(now,'dd_mmm_yyyy_HHMMSS') '.mat'];
% save(matFileName,'-v7.3');

% % to generate the ideal data
targetModel.type = [targetModel.type 'Generative'];
targetModel.generativex0 = targetState.x0;
targetModel.generativex = [];
for k = 1:length(targetStateHist)
    targetModel.generativex = [targetModel.generativex targetStateHist{k}.x];
end
% clearvars -except targetModel
% save('constantSpeedRandomWalkGenerative.mat');
save(['./scenes/targetMotion' num2str(kk) '.mat'],'targetModel');

% close all
% plot(trueWorld.G_env,'XData',trueWorld.G_env.Nodes.x,'YData',trueWorld.G_env.Nodes.y);
% hold on;
% axis equal;
% pause;
%
% for k = 2:301
% handle = plot(trueWorld.nodeX(targetStateHist{k}.x(1)),trueWorld.nodeY(targetStateHist{k}.x(1)),'ro');
% drawnow;pause(0.1);
% set(handle,'Visible','Off');
% end

end
