function [swarmState, swarmWorld] = taskAllocation_decentralized(tasks, swarmState, swarmModel, swarmWorld, trueWorld, simParams)
if strcmp(swarmModel.taskAllocation,'Hungarian')
    % Hungarian algorithm goes here
    
    % Sheng: just combine the energy and penalty method here
    if strcmp(swarmModel.taskGeneration,'frontierWpts')
        for k = 1:swarmModel.N
            % first agents towards the center
            %             swarmState{k}.xd = rand(1,1)*(max(trueWorld.xpoly)-min(trueWorld.xpoly))+min(trueWorld.xpoly);
            %             swarmState{k}.yd = rand(1,1)*(max(trueWorld.ypoly)-min(trueWorld.ypoly))+min(trueWorld.ypoly);
            swarmState{k}.xd = (trueWorld.maxX+trueWorld.minX)/2*ones(size(swarmState{k}.xd));
            swarmState{k}.yd = (trueWorld.maxY+trueWorld.minY)/2*ones(size(swarmState{k}.yd));
        end
        % Then, if task has been generated, then run a task allocation algorithm
        % to assign tasks to agents
        % This approach makes sure all agents are assigned waypoint even if
        % the number of tasks is less than agents
        
        if ~isempty(tasks)
            for k = 1:swarmModel.N
                % modify it just to make sure we get the centralized decision
                swarmModel.communicationTopology = 'centralized';
                
                % if tasks are not empty, then generate the taskTable
                taskTable = generateTaskTable(swarmWorld{k}.allAgentState(:)',tasks,swarmModel,simParams);
                
                
                % do not use frontierandblob for now
                %             if ( strcmp(swarmModel.mapping.method,'frontierAndBlob') )
                %                 % scale the cost for subblob centroid
                %                 taskTable(:,length(swarmWorld.frontierIndex)+1:end) = taskTable(:,length(swarmWorld.frontierIndex)+1:end)*swarmModel.mapping.blobCostScale;
                %             end
                % at this point making a request to auction
                % the return most likely would be the assignment queue or a
                % currently active task.
                %
                % use Munkres method for task allocation
                taskAllocation = munkres(taskTable);
                if taskAllocation(k)~=0
                    swarmState{k}.xd = tasks(taskAllocation(k),1);
                    swarmState{k}.yd = tasks(taskAllocation(k),2);
                end
                %                 [assignedAgent,~] = find(~~taskAllocation');
                %                 if ~isempty(assignedAgent)
                %                     swarmState{assignedAgent(k)}.xd = tasks(taskAllocation(assignedAgent(k))',1);
                %                     swarmState{assignedAgent(k)}.yd = tasks(taskAllocation(assignedAgent(k))',2);
                %                 end
                % resume the option
                swarmModel.communicationTopology = 'allToAll';
            end
        else
            for k = 1:swarmModel.N
                % if there are no tasks, then just assign all agents to the
                % center (or switch to other actions)
                swarmState{k}.xd = (trueWorld.maxX+trueWorld.minX)/2*ones(size(swarmState{k}.xd));
                swarmState{k}.yd = (trueWorld.maxY+trueWorld.minY)/2*ones(size(swarmState{k}.yd));
            end
        end
        
    elseif strcmp(swarmModel.taskGeneration,'voronoiWpts')
        % build a distance matrix
        M = length(tasks);
        D = zeros(swarmModel.N,M);
        for i = 1:1:swarmModel.N % rows are agents
            for j = 1:1:M % columns are task
                D(i,j) = norm(tasks(j,:)-[swarmState.x(4*i-3) swarmState.x(4*i-2)]);
            end
        end
        % build a reward matrix based on cell mass
        R = zeros(swarmModel.N,M);
        for i = 1:1:swarmModel.N
            R(i,:) = swarmWorld.cellMass;
        end
        % Cost
        R = scaleMatrix(R); % scale matrix so smallest entry is 0, largest is 1
        D = scaleMatrix(D); % scale matrix so smallest entry is 0, largest is 1
        C = -R; % ignore distance
        % solve a single-task assignment problem using the munkres/hungarian algorithm
        assignment = munkres(C);
        % set desired waypoints
        swarmState.xd = tasks(assignment,1);
        swarmState.yd = tasks(assignment,2);
    elseif strcmp(swarmModel.taskGeneration,'randomWpts')
        swarmState.xd = tasks(:,1);
        swarmState.yd = tasks(:,2);
    end
    
elseif strcmp(swarmModel.taskAllocation,'Auctioneer')
    
    
    for j = 1:swarmModel.N
        % placeholder for tasks
        swarmWorld{j}.currentTaskList = tasks;
        
        % update location of each agent
        currentState = roboticState();
        currentState.POSITION = swarmWorld{j}.OperationalWorld.loadAgentLocation([swarmState{j}.x(1:2) 0]);
        swarmWorld{j}.roboticAgents.updateRoboticState(currentState);
        
        % update everyTask and availableTask
        if isempty(swarmWorld{j}.everyTask)
            swarmWorld{j}.everyTask = tasks; %everyTask captures everything ever generated
            swarmWorld{j}.availableTask = ones(size(swarmWorld{j}.everyTask,1),1); %updates per condition above
            newTasks = tasks;
        else
            % find out which task in tasks is newly added
            [newTasksIndicator,~] = find(1-ismember(tasks,swarmWorld{j}.everyTask,'rows'));
            % merge the tasks to everyTask
            swarmWorld{j}.everyTask = [swarmWorld{j}.everyTask;tasks(newTasksIndicator,:)];%union(swarmWorld{k}.everyTask,tasks,'rows');
            swarmWorld{j}.availableTask = [swarmWorld{j}.availableTask;ones(length(newTasksIndicator),1)];
            
            % find out the index of the newly added tasks
            [~,newTaskIndex] = ismember(tasks(newTasksIndicator,:),swarmWorld{j}.everyTask,'rows');
            newTasks = swarmWorld{j}.everyTask(newTaskIndex,:);
        end
    end
    
    % count newly added tasks
    numNewTasks = size(newTasks,1); %newTasks
    
    
    % append newly added tasks to availableTasks
    newTaskQueue = ADCA_AvailableTaskQueue();
    for m = 1:numNewTasks
        [~,taskID] = ismember(newTasks(m,:),swarmWorld{1}.everyTask,'rows');
        
        taskContainer = ADCA_TaskContainer();
        
        taskDescriptor = ADCA_TaskLoiterDescriptor(0, taskID);
        taskDescriptor.LoiterLocation = swarmWorld{1}.OperationalWorld.loadTaskLocation([newTasks(m,:) 0]);% Sheng: load locations of tasks
        
        taskContainer.setTaskDescriptor(taskDescriptor);
        newTaskQueue.appendTaskToList(taskContainer);
    end
    
    %     newTaskQueue.printTaskList();
    newTaskQueue;
    
    % Notify all of the agents of the currently available tasks (Sheng: can we change line 175 to just copy availableTasks to each agent?)
    for j = 1:swarmModel.N
        swarmWorld{j}.roboticAgents.newlyAvailableTasks(newTaskQueue);
    end
    
    % move all agents to a place holder: roboticAgentsArray
    roboticAgentsArray = [];
    
    for j = 1:swarmModel.N
        roboticAgentsArray = [roboticAgentsArray;swarmWorld{j}.roboticAgents];
    end
    
    % bid and consensus
    consensusAchieved = false;
    abort = false;
    
    auctionIteration = 1;
    lastAuctionChange = auctionIteration - 1;
    
    % this while-loop remains unchanged to Ken's code
    while ((consensusAchieved == false) && (abort == false))
        %Communicate to all the agents about the current belief
        ADCA_MeshCommunication(roboticAgentsArray,swarmWorld{1}.communicationMatrix);  % ATTN Sheng: swarmWorld{k}.communicationMatrix needs generalizing
        
        %Allow the agents to establish each of their own desired bundles
        for j = 1:swarmModel.N
            [generatedBundle, newlyFormedBundle] = ADCA_BuildBundle(swarmWorld{j}.roboticAgents,datetime('now'),swarmWorld{j}.roboticAgents.getConcludingState(),swarmWorld{j}.roboticAgents.getAvailableTasks());
            if(newlyFormedBundle)
                lastAuctionChange = auctionIteration;
            end
        end
        
        if((auctionIteration - lastAuctionChange) > 0)
            consensusAchieved = true;
%         elseif((auctionIteration - lastAuctionChange) > (100))
%             consensusAchieved = true;
        end
        
        auctionIteration = auctionIteration + 1;
    end
    
    %A consensus has been reached. Based on the formulation of how the
    %auctioneer is implemented in the UMD code base, we need to clear the
    %current bid bundles before the next round
    
    for i = 1:swarmModel.N 
        swarmWorld{i}.roboticAgents.clearCurrentBidBundle();
        newCurrentTask = swarmWorld{i}.roboticAgents.m_CurrentTaskQueue.getCurrentTask();
        if(~isempty(newCurrentTask))
            for j = 1:swarmModel.N
                swarmWorld{j}.roboticAgents.updateTaskAvailability(newCurrentTask.m_TaskDescriptor.getTaskKey(),false);
            end
            swarmState{i}.xd = newCurrentTask.m_TaskDescriptor.LoiterLocation.x;
            swarmState{i}.yd = newCurrentTask.m_TaskDescriptor.LoiterLocation.y;
        end
        
        % save the assigned bundle
        swarmWorld{i}.assignedBundle = swarmWorld{i}.roboticAgents.m_CurrentTaskQueue.getCurrentTaskQueueLocation;
    end
        
    
end
end

