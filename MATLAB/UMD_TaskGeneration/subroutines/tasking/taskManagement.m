function [swarmState] = taskManagement(swarmState, swarmModel, swarmWorld)

% task management
switch swarmModel.taskAllocation
    case {'stepwiseHungarian','stepwiseHungarian_max','stepwiseHungarian_2ndOrder','stepwiseHungarian_unique'}
        for i = 1:1:swarmModel.N
            xi = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i)];
            xd = [ swarmState.xd(i) swarmState.yd(i) ];
            if ( norm([xi(1) xi(2)] - xd) <= swarmModel.Rsense/3*2 ) && (swarmState.wptIndex(i) < length(swarmState.wptList(i,:)))
                swarmState.wptIndex(i) = swarmState.wptIndex(i)+1;
                swarmState.xd(i) = swarmWorld.cellCenterOfMass(swarmState.wptList(i,swarmState.wptIndex(i)),1);
                swarmState.yd(i) = swarmWorld.cellCenterOfMass(swarmState.wptList(i,swarmState.wptIndex(i)),2);
            end
            
        end
    case 'none'
        if strcmp(swarmModel.taskGeneration,'lawnmower')
            % first check if each agent has reached previously assigned
            % waypoint, then only update
            for i = 1:1:swarmModel.N
                xi = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i)];
                xd = [ swarmState.xd(i) swarmState.yd(i) ];
                if ( norm([xi(1) xi(2)] - xd) <= swarmModel.Rsense/10 )
                    disp('Updating Wpt');
                    swarmState.wptIndex(i) = swarmState.wptIndex(i) + 1;
                    if ( swarmState.wptIndex(i) > swarmState.numWpts )
                        swarmState.wptIndex(i) = 1;
                        fprintf('Quad %i completed lawnmower, restarting \n',i);
                    end
                    swarmState.xd(i,1) = swarmState.wptListX(swarmState.wptIndex(i),i);
                    swarmState.yd(i,1) = swarmState.wptListY(swarmState.wptIndex(i),i);
                end
            end
        end
end

end
