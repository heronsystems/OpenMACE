function [rebroadcastBid, invalidate] = ADCA_Consensus_SenderBelievesNone(senderID, senderLeadingBid, receivingAgent, currentTime)

tieEpsilon = 0.1;
currentKey = senderLeadingBid.m_TaskKey;
invalidate = false; %let us first assume that the bid is valid
rebroadcastBid = []; %assume that there will be nothing to rebroadcast

%Grab all the information related to the receiver's information regarding
%this task associated with the current key.
receiverID = receivingAgent.m_RoboticParameters.AGENT_ID;
receivingMasterQueue = receivingAgent.m_MasterTaskList;
receiverTask = receivingMasterQueue.getTaskWithKey(currentKey);
receiverLeadingBid = receiverTask.whatIsLeadingBid();

%Update the time that the receiver will have gotten the latest update with
%this information should it go forward.
senderLeadingBid.updateReceptionTime(currentTime);

if(isempty(receiverLeadingBid))
    %There is nothing to do.
elseif(receiverLeadingBid.AGENT_ID == receiverID)
    %There is nothing to update. The receiving agents bid should be
    %distributed to the collective robotics group.
    rebroadcastBid = receiverLeadingBid;

elseif(receiverLeadingBid.AGENT_ID == senderID)
    %The sender is telling us that the bid is no longer valid.
    receiverTask.invalidateLeadingAgent();
elseif((receiverTask.m_CurrentBids.LEADING_AGENT ~= receiverID) &&...
        (receiverTask.m_CurrentBids.LEADING_AGENT ~= senderID))
    %The receiver believes that an agent that is not themselves or the
    %sender is the winner.
        if(senderLeadingBid.GENERATION_TIME > receiverLeadingBid.GENERATION_TIME)
            receiverTask.invalidateBid(senderID);
            rebroadcastBid = senderLeadingBid;
        end
    
end


end