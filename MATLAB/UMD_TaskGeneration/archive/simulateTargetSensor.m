function [signals] = simulateTargetSensor( swarmState, swarmModel, trueWorld, targetState, targetModel )

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

% initialize outputs
signals = zeros(swarmModel.N,1);
positions = zeros(swarmModel.N,2);

% iterate over each agent
for i = 1:1:swarmModel.N
    
    % get agent xy position
    agentXY = [swarmState.x(4*i-3); swarmState.x(4*i-2)];
    
    % compute distance to each target
    for j = 1:1:targetModel.M
        distToTargs(j) = norm( agentXY' - targetsXY(j,:) );
    end
    
    % check if any target is in view
    targsInView = find( distToTargs <= swarmModel.Rsense );
    
    % if there is a target in view:
    if ( ~isempty(targsInView) )
        % generate signal with given sensitivity
        signals(i) = randn() + swarmModel.m;
    else  % if there is no target in view:
        % pure noise signal for detection/absence
        signals(i) = randn();
    end
end

end
