%MAINTESTSCRIPT this is the main test script program that generates the
%environment for the auctioneer algorithm
%% CLEAR EXISTING ENVIRONMENT AND ESTABLISH NEW ONE

close all;
clear all;
clc;

% plotWorld = figure('Name', 'Initial Agent / Task Environment');
% xlabel('X Position (m)') % x-axis label
% ylabel('Y Position (m)') % y-axis label
% zlabel('Z Position (m)') % z-axis label

% grid on;
% hold on;
agentUtilityMap = containers.Map;

textOffset = 0.25; %distance offset used when labeling the agents/tasks

%% DEFINE THE OPERATIONAL WORLD AVAILABLE TO THE ROBOTIC AGENTS

%Establish the min/max X operational values
minX = -50;
maxX = 50;

%Establish the min/max Y operational values
minY = -50;
maxY = 50;

%Establish the min/max Z operational values
minZ = 0;
maxZ = 10;

%update the graph axis with the operational dimensions
% axis([minX, maxX, minY, maxY, minZ, maxZ])

%Create a new robotic operational environment
OperationalWorld = RobotEnvironment(minX, maxX, minY, maxY, minZ, maxZ);
EnvironmentAxes = OperationalWorld.operationsFigure.CurrentAxes;

%% DEFINE THE AGENTS THAT ARE AVAILABLE WITHIN THE WORLD

numAvailableAgents =3; %maximum number of vehicles going to be allowed to participate

%Create the inverse of a identity matrix for a communication topology. This
%will enable a structure of [0,1;1,0] to reflect that agents cannot
%communicate with themselves (or should say unecessary). For now, we will
%assume that all agents can talk to eachother in a perfect mesh network
%topology. We can later adapt this and it would exist on a per vehicle
%basis. Rows will capture the receiving agent while the columns shall
%capture the sending agent.
communicationMatrix = ~eye(numAvailableAgents);

numPlanes = 0; %counter determining how many planes added to sim
numRotary = 0; %counter determining how many rotary added to sim

for n = 1:numAvailableAgents
    if(rand > 0.5) %add an available plane
        roboticAgents(n,1) = GenericRoboticAgent(n, ADCA_AgentTypes.UAV_ROTARY, OperationalWorld);
    else %add an available quadrotor
        roboticAgents(n,1) = GenericRoboticAgent(n, ADCA_AgentTypes.UAV_ROTARY, OperationalWorld);
    end
    
    robotLocation = OperationalWorld.getRandomLocation(); % getRandomLocation in RobotEnvironment.m
    % Sheng: get agents evenly spaced
%     robotLocation = OperationalWorld.getEquidistantLocation(n,numAvailableAgents,0.8);

    currentState = roboticState();
    currentState.POSITION = robotLocation;
    roboticAgents(n,1).updateRoboticState(currentState);
    
    plot3(EnvironmentAxes, robotLocation.x, robotLocation.y, robotLocation.z, 'o', 'color', OperationalWorld.colorMapVehicles(roboticAgents(n,1).m_RoboticParameters.AGENT_TYPE, :));
    text(robotLocation.x + textOffset, robotLocation.y + textOffset, robotLocation.z + textOffset, ['A' num2str(n)], 'Parent', EnvironmentAxes);
    hold on;
end %end of for loop creating robotic agents

%roboticAgents(1,1).m_RoboticParameters.FuelCost = 0.9*roboticAgents(2,1).m_RoboticParameters.FuelCost;
%if(numPlanes < 1) %ensure that we have atleast 1 plane to service explicit tasks
%    roboticAgents(numAvailableAgents,1) = GenericRoboticAgent(n,ADCA_AgentTypes.UAV_PLANE);
%end
% if(numRotary < 1) %ensure that we have atleast 1 rotary to service explicit tasks
%     replaceAgent = GenericRoboticAgent(n,ADCA_AgentTypes.UAV_ROTARY);
%     currentAgent = roboticAgents(numAvailableAgents,1);
%     replaceAgent.m_CurrentState = currentAgent.m_CurrentState;
%     roboticAgents(numAvailableAgents,1) = replaceAgent;
% end

for n = 1:size(roboticAgents,1)
    if(roboticAgents(n,1).m_RoboticParameters.AGENT_TYPE == ADCA_AgentTypes.UAV_PLANE)
        numPlanes = numPlanes + 1;
    elseif(roboticAgents(n,1).m_RoboticParameters.AGENT_TYPE == ADCA_AgentTypes.UAV_ROTARY)
        numRotary = numRotary + 1;
    end
    
end %end of for loop creating robotic agents
fprintf('The number of available fixed wing vehicles in this environment is: %i\n',numPlanes);
fprintf('The number of available rotary wing vehicles in this environment is: %i\n',numRotary);

%% DEFINE THE TASKS THAT ARE INITIALLY AVAILABLE TO THE ROBOTIC AGENTS

numInitialTasks = 10; %number of initial tasks to seed the environment with

availableTasks = ADCA_AvailableTaskQueue();

for m = 1:numInitialTasks
    taskContainer = ADCA_TaskContainer();
    
    taskDescriptor = ADCA_TaskLoiterDescriptor(0, m);
    taskDescriptor.LoiterLocation = OperationalWorld.getRandomLocation(); % getRandomLocation in RobotEnvironment.m
    % Sheng: simple even spaced test
%     taskDescriptor.LoiterLocation = OperationalWorld.getEquidistantLocation(m,numInitialTasks,0.2);

    taskContainer.setTaskDescriptor(taskDescriptor);
    %taskContainer.m_BeginningState.POSITION = OperationalWorld.getRandomLocation();
    %taskContainer.m_EndingState.POSITION = OperationalWorld.getRandomLocation();
    
    availableTasks.appendTaskToList(taskContainer);
    
    taskDescriptor.graphTaskProperties(EnvironmentAxes, OperationalWorld.colorMapTasks);
    
    %     startTask = taskContainer.m_BeginningState.POSITION;
    %     plot3(EnvironmentAxes, startTask.x, startTask.y, startTask.z,'+','color',OperationalWorld.colorMapTasks(ADCA_TaskTypes.TASK_LOITER,:));
    %     text(startTask.x + textOffset, startTask.y + textOffset, startTask.z + textOffset, ['T' num2str(m)], 'Parent', EnvironmentAxes);
end

%Notify all of the agents of the currently available tasks
for n = 1:numAvailableAgents
    roboticAgents(n,1).newlyAvailableTasks(availableTasks);
end

%% DEFINE THE TASKS THAT ARE INITIALLY AVAILABLE TO THE ROBOTIC AGENTS

consensusAchieved = false;
abort = false;

auctionIteration = 1;
lastAuctionChange = auctionIteration - 1;

while ((consensusAchieved == false) && (abort == false))
    %Communicate to all the agents about the current belief
    ADCA_MeshCommunication(roboticAgents,communicationMatrix);
    
    for n = 1:numAvailableAgents
        fprintf("The perceived leaders of agent %i. \n",roboticAgents(n,1).m_RoboticParameters.AGENT_ID);
        roboticAgents(n,1).m_MasterTaskList.printTaskLeaders();
        instantaneousAgentUtility = roboticAgents(n,1).m_MasterTaskList.computeAgentUtility(n);
        if(isKey(agentUtilityMap,num2str(n)))
            agentUtilityMap(num2str(n)) = [agentUtilityMap(num2str(n)),instantaneousAgentUtility];
        else
            agentUtilityMap(num2str(n)) = instantaneousAgentUtility;
        end
    end
    
    fprintf('We are about to build the appropriate bundles.\n');
    %Allow the agents to establish each of their own desired bundles
    for n = 1:numAvailableAgents
        [generatedBundle, newlyFormedBundle] = ADCA_BuildBundle(roboticAgents(n,1),datetime('now'),roboticAgents(n,1).getConcludingState(),roboticAgents(n,1).getAvailableTasks());
        
        if(newlyFormedBundle)
            lastAuctionChange = auctionIteration;
        end
    end
    
    for n = 1:numAvailableAgents
        fprintf("The perceived task order of agent %i. \n",roboticAgents(n,1).m_RoboticParameters.AGENT_ID);
        roboticAgents(n,1).m_MasterTaskList.printCurrentAssignments();
    end
    
    if((auctionIteration - lastAuctionChange) > 0)
        consensusAchieved = true;
    elseif((auctionIteration - lastAuctionChange) > (2*numInitialTasks))
        consensusAchieved = true;
    end
    
    fprintf('Pause after auction iteration %i. \n',auctionIteration);
    auctionIteration = auctionIteration + 1;
    
%     pause;
end


for n = 1:numAvailableAgents
    fprintf('Agent %i current task queue:',n)
    roboticAgents(n,1).m_CurrentTaskQueue.printCurrentTaskQueue();
    fprintf('\n\r');
    
    agentPath = roboticAgents(n,1).graphCurrentTaskPath();
    plot3(EnvironmentAxes, agentPath(:,1), agentPath(:,2), agentPath(:,3));
end


agentSchedules = figure('Name','Agent Scheduling');
subplot(numAvailableAgents,1,1);
title('Agent Scheduling');
for n = 1:numAvailableAgents
    subplot(numAvailableAgents,1,n);
    xlabel('Time (sec)'); % x-axis label
    ylabel(['Agent: #' num2str(n)]); % y-axis label
    grid on;
    hold on;
    roboticAgents(n,1).plotCurrentAgentSchedule(agentSchedules.CurrentAxes);
end

agentUtility = figure('Name','Agent Utility');

subplot(2,1,1);
title('Individual Agent Utility');
xlabel('Auction Iteration'); % x-axis label
ylabel('Utility'); % y-axis label
grid on;
hold on;
for k = keys(agentUtilityMap)
    theKey = k{1};
    agentUtilityPerIteration = agentUtilityMap(theKey);
    plot(agentUtilityPerIteration);
end
set(gca,'xtick',[1:1:auctionIteration-1])

    
subplot(2,1,2);
title('Global Agent Utility');
xlabel('Auction Iteration'); % x-axis label
ylabel('Normalized Utility'); % y-axis label
grid on;
hold on;
globalUtility = 0;
for idx = 1:(auctionIteration-1)
    iterationUtility = 0;
    for k = keys(agentUtilityMap)
        theKey = k{1};
        agentUtilityPerIteration = agentUtilityMap(theKey);
        iterationUtility = iterationUtility + agentUtilityPerIteration(1,idx);
    end
    globalUtility(1,idx) = iterationUtility;
end
set(gca,'xtick',[1:1:auctionIteration-1])
globalUtility = globalUtility/max(globalUtility);
plot(globalUtility)