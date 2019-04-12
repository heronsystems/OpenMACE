function [] = ADCA_UpdateBundle(roboticAgent)

agentID = roboticAgent.m_RoboticParameters.AGENT_ID;

taskList = roboticAgent.m_CurrentTaskQueue;

masterList = roboticAgent.m_MasterTaskList;

agentLostAuction = false;

j = 1;
while j <= taskList.sizeOfTaskList()
    taskAtIndex = taskList.getTaskAtIndex(j);
    currentKey = taskAtIndex.m_TaskDescriptor.getTaskKey();
    leaderID = masterList.getTaskWithKey(currentKey).whatIsLeadingBid().AGENT_ID;
    if(leaderID ~= agentID)
        agentLostAuction = true;
    end
    
    if(agentLostAuction)
        %because the agent lost the task, it and everything after the
        %task is released
        taskList.removeTaskAtIndex(j);
        if(leaderID == agentID)
            masterList.agentLostTask(agentID,currentKey);
            %masterList.getTaskWithKey(currentKey).m_CurrentBids.invalidateLeadingAgent();
        end
    else
        j = j + 1;
    end
end

end