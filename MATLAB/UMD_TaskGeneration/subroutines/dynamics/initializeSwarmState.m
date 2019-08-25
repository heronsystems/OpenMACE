function [swarmState, ROS_MACE] = initializeSwarmState(swarmModel, trueWorld, runParams, ROS_MACE)
if ( strcmp(runParams.type, 'mace') )
    ROS_MACE = setupF3FlightTestPlot( runParams,ROS_MACE );
    ROS_MACE = launchROS( ROS_MACE );
    disp('Press key when IMU01 using GPS across all quads');
    pause;
    disp('Setting datum');
    swarmState = sendDatumAndWaitForGPS( ROS_MACE );
    
    if strcmp(ROS_MACE.wptCoordinator,'standalone')
        %     !matlab -r standaloneWptCoordinator 120 &
        eval(['!matlab -desktop -r ''standaloneWptCoordinator(' num2str(runParams.T) ',' num2str(swarmModel.Rsense) ',' num2str(swarmModel.N) ',[' num2str(ROS_MACE.agentIDs) '])'' &']);
        %eval(['!matlab -desktop -r ''standaloneWptCoordinator(' num2str(runParams.T) ')'' &']);
        % -desktop will start a new MATLAB window for debugging
        fprintf('***Wait for the StandalongWptCoordinator to start*** \n');
        countdownVerbose(3);
    end
    
    armAndTakeoff( ROS_MACE );
    if ( ROS_MACE.startOnPerimeter )
        dispatchSwarmToPerimeter( ROS_MACE , trueWorld );
    end
    disp('Press a key to begin the run...')
    pause;
    countdownVerbose(3);
    % start another matlab instance to run the wpt coordinator
    
    
    
elseif ( strcmp(runParams.type, 'matlab') )
    swarmModel.taskGeneration
    if ( strcmp(swarmModel.taskGeneration,'lawnmower')==1 )
        % we *require *4 agents for the lawnmower
        if ( swarmModel.N ~= 4 )
            error('Must have 4 agents with taskGeneration = lawnmower');
        end
        R = swarmModel.Rsense;
        xmin = trueWorld.minX;
        ymin = trueWorld.minY;
        xmax = trueWorld.maxX;
        ymax = trueWorld.maxY;
        
        % agents arranged clockwise starting from lower left corner
        R = R*0.95;
        [xpts1, ypts1, xinit1, yinit1] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 1, R);
        [xpts2, ypts2, xinit2, yinit2] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 2, R);
        [xpts3, ypts3, xinit3, yinit3] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 3, R);
        [xpts4, ypts4, xinit4, yinit4] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 4, R);
        swarmState.numWpts = length(xpts1);

        xv = [xinit1 xinit2 xinit3 xinit4];
        yv = [yinit1 yinit2 yinit3 yinit4];
        
        swarmState.wptIndex(1) = 1;
        swarmState.wptIndex(2) = 1;
        swarmState.wptIndex(3) = 1;
        swarmState.wptIndex(4) = 1;
        
        swarmState.wptListX(:,1) = xpts1;
        swarmState.wptListX(:,2) = xpts2;
        swarmState.wptListX(:,3) = xpts3;
        swarmState.wptListX(:,4) = xpts4;
        
        swarmState.wptListY(:,1) = ypts1;
        swarmState.wptListY(:,2) = ypts2;
        swarmState.wptListY(:,3) = ypts3;
        swarmState.wptListY(:,4) = ypts4;
        
        swarmState.xd(1,1) = swarmState.wptListX(swarmState.wptIndex(1),1);
        swarmState.xd(2,1) = swarmState.wptListX(swarmState.wptIndex(2),2);
        swarmState.xd(3,1) = swarmState.wptListX(swarmState.wptIndex(3),3);
        swarmState.xd(4,1) = swarmState.wptListX(swarmState.wptIndex(4),4);
        
        swarmState.yd(1,1) = swarmState.wptListY(swarmState.wptIndex(1),1);
        swarmState.yd(2,1) = swarmState.wptListY(swarmState.wptIndex(2),2);
        swarmState.yd(3,1) = swarmState.wptListY(swarmState.wptIndex(3),3);
        swarmState.yd(4,1) = swarmState.wptListY(swarmState.wptIndex(4),4);
        
    else
        % initialize agent states
        % distribute the agents uniformly along the boundary,
        % prevPt indicates a node on the (xpoly,ypoly) that is closest to the agent
        
        if ( swarmModel.useGeneratedInitialFormation && (swarmModel.initialFormationID ~= -1) )% if this is 1, then use the generated formation
            idx = swarmModel.initialFormationID;
            load(['./scenes/initialFormation' num2str(idx) '.mat']);
            disp('loading initial formation from mat file');
        elseif ( swarmModel.useGeneratedInitialFormation && (swarmModel.initialFormationID == -1) )
            if (swarmModel.N == 4)
                % use lawnmower function to determine corner pts
                % (ignore
                R = swarmModel.Rsense;
                xmin = trueWorld.minX;
                ymin = trueWorld.minY;
                xmax = trueWorld.maxX;
                ymax = trueWorld.maxY;
                [~,~, xinit1, yinit1] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 1, R);
                [~,~, xinit2, yinit2] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 2, R);
                [~,~, xinit3, yinit3] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 3, R);
                [~,~, xinit4, yinit4] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 4, R);
                xv = [xinit1 xinit2 xinit3 xinit4];
                yv = [yinit1 yinit2 yinit3 yinit4];
                disp('Using Corner pts for initial formation')
            else
                error('Error: Corner initial formation requires 4 agents');
            end
        else
            [xv, yv, ~] = distributeUniformlyAlongCurve(swarmModel.N,trueWorld.xpoly,trueWorld.ypoly);
            % initialize agent desired waypoints
            % the task assignment problem will update the 'swarmState' for each agent
            swarmState.xd = rand(swarmModel.N,1)*(max(trueWorld.xpoly)-min(trueWorld.xpoly))+min(trueWorld.xpoly);
            swarmState.yd = rand(swarmModel.N,1)*(max(trueWorld.ypoly)-min(trueWorld.ypoly))+min(trueWorld.ypoly);
        end
    end
    % each agent has states [x y xdot ydot]
    for i = 1:1:swarmModel.N
        swarmState.x0(4*i-3,1) = xv(i);
        swarmState.x0(4*i-2,1) = yv(i);
        swarmState.x0(4*i-1,1) = 0;
        swarmState.x0(4*i,1) = 0;
    end
    swarmState.x(1,:) = swarmState.x0';
    
    % initialize simulation time
    swarmState.t = 0;
    swarmState.k = 0;
end
