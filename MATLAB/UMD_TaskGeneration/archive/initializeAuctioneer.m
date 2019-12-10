function [swarmWorld] = initializeAuctioneer(swarmWorld)

cd ..
cd 'Auctioneer Strategy Matlab'/
addpath(pwd);
cd ..
cd UMD_TaskGeneration/

%Establish the min/max X operational values
minX = -50;
maxX = 50;

%Establish the min/max Y operational values
minY = -50;
maxY = 50;

%Establish the min/max Z operational values
minZ = 0;
maxZ = 10;

swarmWorld.agentUtilityMap = containers.Map;
%Create a new robotic operational environment
swarmWorld.OperationalWorld = RobotEnvironment(minX, maxX, minY, maxY, minZ, maxZ);
swarmWorld.EnvironmentAxes = swarmWorld.OperationalWorld.operationsFigure.CurrentAxes;
swarmWorld.communicationMatrix = ~eye(swarmModel.N);

swarmWorld.communicationMatrix = ~eye(swarmModel.N);

swarmWorld.agentUtilityMap = containers.Map;

for n = 1:swarmModel.N
    swarmWorld.roboticAgents(n,1) = GenericRoboticAgent(n, ADCA_AgentTypes.UAV_ROTARY, swarmWorld.OperationalWorld);
    
    robotLocation = swarmWorld.OperationalWorld.getRandomLocation(); % getRandomLocation in RobotEnvironment.m
    % Sheng: get agents evenly spaced
    % robotLocation = OperationalWorld.getEquidistantLocation(n,numAvailableAgents,0.8);
    
    currentState = roboticState();
    currentState.POSITION = robotLocation;
    swarmWorld.roboticAgents(n,1).updateRoboticState(currentState);
    
    %     	plot3(EnvironmentAxes, robotLocation.x, robotLocation.y, robotLocation.z, 'o', 'color', OperationalWorld.colorMapVehicles(roboticAgents(n,1).m_RoboticParameters.AGENT_TYPE, :));
    %     	text(robotLocation.x + textOffset, robotLocation.y + textOffset, robotLocation.z + textOffset, ['A' num2str(n)], 'Parent', EnvironmentAxes);
    hold on;
end %end of for loop creating robotic agents
end
