function [rebroadcastBid, invalidate] = ADCA_Consensus_SenderBelievesI(senderID, senderLeadingBid, receivingAgent, currentTime)

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

if(senderLeadingBid.AGENT_ID == receiverID)
    %Check to see if the receiver had assigned an initial leader
    if(isempty(receiverLeadingBid))
        %The receiver had not assigned a leader, we therefore do not
        %understand how it got to this state. We should send a clear
        %message.
        clearBid = ADCA_BidDescriptor();
        clearBid.m_TaskKey = currentKey;
        clearBid.useAsClearingBid(true);
        clearBid.GENERATION_TIME = currentTime;
        clearBid.UPDATE_TIME = currentTime;
        rebroadcastBid = clearBid;
    elseif(receiverLeadingBid.AGENT_ID == receiverID)
        %The receiver believes it is the winner and the sender also has
        %this notion...there is nothing to do.
    elseif(receiverLeadingBid.AGENT_ID == senderID)
        %There is a complete disconnect between agents. Each
        %had thought the other had actually won the task.
        %Therefore, we just need to clear the leading agent for
        %this task and move on.
        receiverTask.invalidateBid(senderID);
        
        clearBid = ADCA_BidDescriptor();
        clearBid.m_TaskKey = currentKey;
        clearBid.useAsClearingBid(true);
        clearBid.GENERATION_TIME = currentTime;
        clearBid.UPDATE_TIME = currentTime;
        rebroadcastBid = clearBid;
    else
        %The receiver thought that the winner was neither the sender and/or
        %the receiver. Since no conclusive descision can be made given this
        %condition, we shall redsit
        rebroadcastBid = receiverLeadingBid;
    end
end %end of if statement ensuring sender selected the receiver to win

end