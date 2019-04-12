function [biddingBundle, newBid] = ADCA_BuildBundle(roboticAgent, currentTime, startingState, availableTasks)

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

biddingBundle = ADCA_BiddingBundle();
newBid = false;

if(availableTasks.sizeOfTaskList() < 1)
    return;
end

startingState = roboticAgent.getConcludingState();
% position = startingState.POSITION.getPositionVector()';
CurrentAxes = roboticAgent.biddingBundleFigure.CurrentAxes;
currentTaskQueueSize = roboticAgent.m_CurrentTaskQueue.sizeOfTaskList();

%Clear the current axes to plot the next bundle
cla(CurrentAxes); 
while ((availableTasks.sizeOfTaskList() > 0) && (~biddingBundle.isBundleSizeExceeded(roboticAgent.m_AuctionParameters.MAX_BUNDLESIZE)) &&...
        ((currentTaskQueueSize + biddingBundle.getBiddingBundleSize) < roboticAgent.m_AuctionParameters.MAX_ASSIGNMENTSIZE))
    
    [leadingBid, concludingState, leadingIndex] = ADCA_ComputeBidEstimates(roboticAgent, currentTime, biddingBundle, startingState, availableTasks);
    
    if(isempty(leadingBid))
        break;
    else
        newBid = true;
    end
    
    leadingTask = availableTasks.removeTaskAtIndex(leadingIndex);
    biddingBundle.appendNewBid(leadingBid);
    
%     plotIndex = size(position,1) + 1;
%     positionStart = leadingBid.m_BeginningState.POSITION.getPositionVector()';
%     positionEnd = leadingBid.m_EndingState.POSITION.getPositionVector()';
%     
%     position(plotIndex,:) = positionStart;
%     position(plotIndex+1,:) = positionEnd;
%     text(position(plotIndex,1),position(plotIndex,2),position(plotIndex,3), ['T' num2str(leadingTask.m_TaskDescriptor.getTaskID())], 'Parent', CurrentAxes);
%     plot3(CurrentAxes, position(plotIndex,1), position(plotIndex,2), position(plotIndex,3), '+');
%     plot3(CurrentAxes, position(plotIndex+1,1),position(plotIndex+1,2),position(plotIndex+1,3),'+');
    
    startingState = concludingState;
end
% plot3(CurrentAxes, position(1,1),position(1,2),position(1,3),'o');
% plot3(CurrentAxes, position(:,1),position(:,2),position(:,3));

%In this step, if the agent has a bundle that they are proposing to work
%on, we can assume it is valid and perform a self assignment
if(~isempty(biddingBundle))
    roboticAgent.updateHostBiddingBundle(biddingBundle);
end
end