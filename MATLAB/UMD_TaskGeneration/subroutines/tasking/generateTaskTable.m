function taskTable = generateTaskTable(agentCurrentState,taskList,swarmModel,simParams,swarmWorld,trueWorld)
%
% Description: generate a table of cost between each agent and each task
%
% Input:
%   agentCurrentState : (1 x (Nagent x 4)) vector, every four-column represents
%   [x y xdot ydot] of agent 1-Nagent
%   taskList : (Ntask x 2) matrix of waypoints for Ntask number of tasks
%   swarmModel: a struct to hold parameters
%   simParams: a struct to hold parameters
% Output:
%   taskTable : (Nagent x Ntask) of cost for each agent to complete each
%   task
%
% Notes: cost measured by agent's energy to reach target +
% penalty proportional to remaining distance between agent and target
%
%
% Sheng Cheng, 2018
Nagent = swarmModel.N;

Ntask = size(taskList,1);

taskTable = zeros(Nagent,Ntask);

% reshape the agent state vector to make it easier to work with.
% careful to use reshape function!!!!
agentCurrentState = reshape(agentCurrentState,[4,Nagent])';

for k = 1:Nagent
    for j = 1:Ntask
        % utility is computed by negating the information gain (so the task with largest information gain is chosen)
        taskTable(k,j) = -computeInformationGain(agentCurrentState(k,:),taskList(j,:),swarmModel,simParams,swarmWorld.mutualInfoSurface,trueWorld);
    end
end
