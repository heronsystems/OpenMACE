function targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams, time)
if ( strcmp(targetModel.type, 'varyingSpeedRandomWalk') )
    for i = 1:1:targetModel.M % xt = [curNode, lastNode, restTime, timeAtNode]
        curNode = targetState.x(4*i-3,1);
        lastNode = targetState.x(4*i-2,1);
        restTime = targetState.x(4*i-1,1);
        timeAtNode = targetState.x(4*i,1);
        % once the timeAtNode surpases the restTime, the target makes a
        % move
        if ( timeAtNode >= restTime )
            % get list of possible moves (adjacent nodes)
            [~,newNodes] =  outedges(trueWorld.G_env, curNode);
            if ( length(newNodes) == 1 ) % reached a dead-end
                lastNode = curNode;
                curNode = newNodes;
                timeAtNode = 0.0; % reset time
            else
                % eliminate lastNode from edge set
                ind = find( lastNode == newNodes );
                newNodes(ind) = [];
                % draw a random new node to visit
                r = randi( length(newNodes) );
                lastNode = curNode;
                curNode = newNodes(r);
                timeAtNode = 0.0;
            end
        else
            % if it not time to move yet, update the timeAtNode
            timeAtNode = timeAtNode + runParams.dt;
        end
        targetState.x(4*i-3,1) = curNode;
        targetState.x(4*i-2,1) = lastNode;
        targetState.x(4*i-1,1) = restTime;
        targetState.x(4*i,1) = timeAtNode;
    end
elseif( strcmp(targetModel.type, 'constantSpeedRandomWalk') )
    for i = 1:1:targetModel.M % xt = [curNode, lastNode]
        curNode = targetState.x(2*i-1,1);
        lastNode = targetState.x(2*i,1);
        if ( mod( targetState.k , floor(targetModel.restPeriod/runParams.Tsamp) ) == 0 )
            % roll the dice to see if target stops
            if ( rand() <= targetModel.probStopping )
                lastNode = curNode;
            else
                % get list of possible moves (adjacent nodes)
                [~,newNodes] =  outedges(trueWorld.G_env, curNode);
                
                % get current heading
                headingCur = atan2( trueWorld.nodeY(curNode) - trueWorld.nodeY(lastNode) , ...
                                    trueWorld.nodeX(curNode) - trueWorld.nodeX(lastNode) );
                                
                % calculate heading along new directions
                headingDiff = zeros(1,length(newNodes));
                for j = 1:1:length(newNodes)
                    headingNew = atan2( trueWorld.nodeY(newNodes(j)) - trueWorld.nodeY(curNode) , ...
                                        trueWorld.nodeX(newNodes(j)) - trueWorld.nodeX(curNode) );
                    headingDiff(j) = angularDist(headingCur, headingNew);             
                end
                
                % calculate the weights
                weights = headingDiff*(-targetModel.inertia/pi) + targetModel.inertia + 1;
                
                % normalize the weights
                weights = weights / sum(weights);
                
                % create a cumulative probability function
                cpf = weights(1);
                for j = 2:1:length(weights)
                    cpf(j) = cpf(j-1) + weights(j);
                end
                
                % draw a random number
                r = rand();
                
                % determine which move to make
                moveIndex = 1;
                while ( r  > cpf(moveIndex) )
                   moveIndex = moveIndex + 1; 
                end
                
                % update 
                lastNode = curNode;
                curNode = newNodes(moveIndex);  
                
            end            
        end
        targetState.x(2*i-1,1) = curNode;
        targetState.x(2*i,1) = lastNode;
    end
    elseif( strcmp(targetModel.type, 'constantSpeedRandomWalkGenerative') )
        % directly load target state from targetModel.generativex
        for i = 1:1:targetModel.M % xt = [curNode, lastNode]
            targetState.x(2*i-1,1) = targetModel.generativex(2*i-1,time);
            targetState.x(2*i,1) = targetModel.generativex(2*i,time);
        end
end
