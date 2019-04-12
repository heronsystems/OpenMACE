function [leadingBid, concludingState, leadingIndex] = ADCA_ComputeBidEstimates(roboticAgent, currentTime, currentBundle, startingState, availableTasks)

%{
    Variable: roboticAgent is of class type GenericRoboticAgent. This
    variable should contain information relating to the parameters of the
    agent, its auction parameters, its current task queue (tasks that the
    agent is currently going to service), and its current state.

    Variable: currentTime is the current time of the simulation environment.
    This variable will be assigned to each of the generated bids in order
    to perform and synchronization and/or auction disagreements in order to
    formulate a consensus.

    Variable: availablTasks is a queue or list of available tasks that an
    agent could service.
%}

%The inner for loop will iterate through all of the available tasks and
%determine its ability to address it and add utility to the collaborative
%group. Tasks that offer a better current utility will be stored as the
%current leader.

leadingBid = [];
concludingState = [];
leadingIndex = 0;

for m = 1:availableTasks.sizeOfTaskList()
    
    evaluatingTask = availableTasks.getTaskAtIndex(m);
    
    %check to see if the current agent can actually service the task...this
    %should be done well prior to this point but for now we will have this
    %check here
    if(~evaluatingTask.m_TaskDescriptor.canAgentAddressTask(roboticAgent.m_RoboticParameters))
        continue;
    end
    
    
    %We could double check to make sure the current queue does
    %not already contain the underlying tasks, but removing the current
    %winning bid seems easier
    
    %To compute the cost we need the expected state (position and time) the
    %agent would be in prior to starting the task and the actual task
    [agentBid, bidConcludingState, feasible] = ADCA_CalculateUtility_UMD(roboticAgent, currentTime, evaluatingTask, startingState);
    
    %If the current task was deemed infeasible let us skip this iteration
    %and move onto the next task.
    if(~feasible)
        continue;
    end
    
    
    %If the task would exceed the current workload lets not allow this
    %task to be apart of the bid regardles 
    
    %if the newly generated bid is greater than the currently calculated
    %leading bid, further consider it for evaluation. if the newly
    %generated bid is greater than any currently received bids then it
    %definitely is a feasible task. Let us establish this as our current
    %leader.
%     if(agentBid.isBidGreater(leadingBid) && evaluatingTask.isBidCompetitive(agentBid))
%         %in this condition this is our new current leader
%         leadingBid = agentBid; %update the current leading bid estimate
%         leadingIndex = m;
%         concludingState = bidConcludingState;
%     end
        
    if(agentBid.isWarpedBidGreater(leadingBid))
        if(evaluatingTask.isBidCompetitive(agentBid))
            %in this condition this is our new current leader
            leadingBid = agentBid; %update the current leading bid estimate
            leadingIndex = m;
            concludingState = bidConcludingState;
        end
    end
    
end

end