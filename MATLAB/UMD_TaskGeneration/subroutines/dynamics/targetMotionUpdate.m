function targetState = targetMotionUpdate(targetState, targetModel, trueWorld, runParams, integerTime)
if( strcmp(targetModel.type, 'constantSpeedRandomWalk') )
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
                headingNew = atan2( trueWorld.nodeY(newNodes) - trueWorld.nodeY(curNode) , ...
                                    trueWorld.nodeX(newNodes) - trueWorld.nodeX(curNode) );
                                
                % calculate the weights
                %weights = angularDist(headingCur, headingNew)*(-targetModel.inertia/pi) + targetModel.inertia + 1;
                
                headingDiff = angularDist(headingCur, headingNew);
                weights = relWeightFun(headingDiff,targetModel.m,targetModel.d);
            
                % create a cumulative probability function of normalized weights
                cpf = cumsum(weights/ sum(weights));
                                                
                % determine which move to make
                r = rand(); % draw a random number
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
            targetState.x(2*i-1,1) = targetModel.generativex(2*i-1,integerTime);
            targetState.x(2*i,1) = targetModel.generativex(2*i,integerTime);
        end
end
