function swarmState = taskAllocation_allToAll_verification(tasks, swarmState, swarmModel, swarmWorld, trueWorld, simParams)
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
                        
	elseif strcmp(swarmModel.taskGeneration,'randomWpts')
    	swarmState.xd = tasks(:,1);
    	swarmState.yd = tasks(:,2);
	end
		
elseif strcmp(swarmModel.taskAllocation,'Auctioneer')
		    
	% wrapper for Ken's auctioneer algorithm goes here
		    
    % MEMO:
    % 1. A task manager is needed and required to
    %   - assign unique id's to tasks
    %   - keep track of the tasks in an agent's bid bundle that is/will be serviced
    % 2. Inside the wrapper, a copy of agents and tasks is maintained with
    %    the variable definition for the auctioneer algorithm by Ken.
		    
    numAvailableAgents = swarmModel.N;
		    
    % get available tasks
    numTasks = size(tasks,1); %number of initial tasks to seed the environment with
		
   	availableTasks = ADCA_AvailableTaskQueue();
		    	    
	for m = 1:numTasks
    	taskContainer = ADCA_TaskContainer();
			    
    	taskDescriptor = ADCA_TaskLoiterDescriptor(0, m);
    	taskDescriptor.LoiterLocation = swarmWorld.OperationalWorld.loadTaskLocation([tasks(m,:) 0]);% Sheng: load locations of tasks
			
    	taskContainer.setTaskDescriptor(taskDescriptor);
    	%taskContainer.m_BeginningState.POSITION = OperationalWorld.getRandomLocation();
    	%taskContainer.m_EndingState.POSITION = OperationalWorld.getRandomLocation();
			    
    	availableTasks.appendTaskToList(taskContainer);
			    
		%     taskDescriptor.graphTaskProperties(EnvironmentAxes, OperationalWorld.colorMapTasks);
    	end
    	%Notify all of the agents of the currently available tasks
    	for n = 1:numAvailableAgents
        	swarmWorld.roboticAgents(n,1).newlyAvailableTasks(availableTasks);
    	end
		    
    	% get available agents: right now assume all the agents are available
		    
    	% bid and consensus
    	consensusAchieved = false;
		abort = false;
			
		auctionIteration = 1;
		lastAuctionChange = auctionIteration - 1;
		    
		while ((consensusAchieved == false) && (abort == false))
    		fprintf('\nWe are about to communicate and perform a consensus step\n');
    		%Communicate to all the agents about the current belief
    		ADCA_MeshCommunication(swarmWorld.roboticAgents,swarmWorld.communicationMatrix);
			    
    		for n = 1:numAvailableAgents
        		fprintf('The perceived leaders of agent %i. \n',swarmWorld.roboticAgents(n,1).m_RoboticParameters.AGENT_ID);
        		swarmWorld.roboticAgents(n,1).m_MasterTaskList.printTaskLeaders();
        		instantaneousAgentUtility = swarmWorld.roboticAgents(n,1).m_MasterTaskList.computeAgentUtility(n);
        		if(isKey(swarmWorld.agentUtilityMap,num2str(n)))
            		swarmWorld.agentUtilityMap(num2str(n)) = [swarmWorld.agentUtilityMap(num2str(n)),instantaneousAgentUtility];
        		else
            		swarmWorld.agentUtilityMap(num2str(n)) = instantaneousAgentUtility;
        		end
    		end
			    
    		fprintf('We are about to build the appropriate bundles.\n');
    		%Allow the agents to establish each of their own desired bundles
    		for n = 1:numAvailableAgents
        		[generatedBundle, newlyFormedBundle] = ADCA_BuildBundle(swarmWorld.roboticAgents(n,1),datetime('now'),swarmWorld.roboticAgents(n,1).getConcludingState(),swarmWorld.roboticAgents(n,1).getAvailableTasks());			        
        		if(newlyFormedBundle)
            		lastAuctionChange = auctionIteration;
        		end
    		end
			    
    		for n = 1:numAvailableAgents
        		fprintf('The perceived task order of agent %i. \n',swarmWorld.roboticAgents(n,1).m_RoboticParameters.AGENT_ID);
        		swarmWorld.roboticAgents(n,1).m_MasterTaskList.printCurrentAssignments();
    		end
			    
    		if((auctionIteration - lastAuctionChange) > 0)
        		consensusAchieved = true;
    		elseif((auctionIteration - lastAuctionChange) > (2*numTasks))
        		consensusAchieved = true;
    		end
			    
    		fprintf('Pause after auction iteration %i. \n',auctionIteration);
    		auctionIteration = auctionIteration + 1;
			    
		end
	end
end

