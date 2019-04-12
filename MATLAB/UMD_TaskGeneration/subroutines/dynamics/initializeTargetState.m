function targetState = initializeTargetState(trueWorld, targetModel)
% initialize target states
targetState.k = 0;
for i = 1:1:targetModel.M
    if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
        % xt = [curNode, lastNode, restTime, timeAtNode]
        targetState.x0(4*i-3,1) = randi(trueWorld.numNodes);
        curNode = targetState.x0(4*i-3,1);
        % having selected the i-th targets curNode, we select a prevNode based
        % on the edges at curNode
        [~,prevNodes] = outedges(trueWorld.G_env, curNode);
        targetState.x0(4*i-2,1) = prevNodes( randi( length(prevNodes) ) );
        targetState.x0(4*i-1,1) = rand()*targetModel.maxSpeed ;
        targetState.x0(4*i,1) = 0.0;
    elseif( strcmp(targetModel.type, 'constantSpeedRandomWalk') )
        targetState.x0(2*i-1,1) = randi(trueWorld.numNodes);
        curNode = targetState.x0(2*i-1,1);
        [~,prevNodes] = outedges(trueWorld.G_env, curNode);
        targetState.x0(2*i,1) = prevNodes( randi( length(prevNodes) ) );
    elseif( strcmp(targetModel.type, 'constantSpeedRandomWalkGenerative') )
        targetState.x0 = targetModel.generativex0;
    end
end
targetState.x(:,1) = targetState.x0;

end