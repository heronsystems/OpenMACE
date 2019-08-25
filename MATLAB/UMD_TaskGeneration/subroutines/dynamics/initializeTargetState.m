function targetState = initializeTargetState(trueWorld, targetModel)
% initialize target states
targetState.k = 0;
for i = 1:1:targetModel.M
    if( strcmp(targetModel.type, 'constantSpeedRandomWalk') )
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