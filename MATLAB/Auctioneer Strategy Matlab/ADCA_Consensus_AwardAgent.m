function [] = ADCA_Consensus_AwardAgent(leadingBid,affectedAgent)

affectedMasterQueue = affectedAgent.m_MasterTaskList;

biddingID = leadingBid.AGENT_ID;
currentKey = leadingBid.m_TaskKey;
affectedID = affectedAgent.m_RoboticParameters.AGENT_ID;
affectedTask = affectedMasterQueue.getTaskWithKey(currentKey);
affectedTask.receivedAdditionalBid(leadingBid);

[lostAgentID, lostTaskIDs] = affectedMasterQueue.awardAgentTask(biddingID,currentKey);
if(lostAgentID == affectedID)
    affectedAgent.m_CurrentTaskQueue.removeTaskAtKey(lostTaskIDs);
end

end

