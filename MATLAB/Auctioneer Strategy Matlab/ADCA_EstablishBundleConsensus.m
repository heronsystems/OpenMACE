function [rebroadcastMsgs, invalidate] = ADCA_EstablishBundleConsensus(senderID, receivedBundle, receivingAgent, currentTime)

rebroadcastMsgs = [];
invalidate = false;

receivingMasterQueue = receivingAgent.m_MasterTaskList;

%For each task the sender has let us go forth and check against the
%receiving agents awareness. Each task has an index of bundleIndex
bundleIndex = 1;
receivedTaskKeys = receivedBundle.getTaskKeys();

while ((bundleIndex <= length(receivedTaskKeys)) && (~invalidate))
    
    senderLeadingBid = receivedBundle.getBidAtIndex(bundleIndex);
    [rebroadcastMsg, invalidate] = ADCA_EvaluateRules(senderID, senderLeadingBid, receivingAgent, currentTime);
    rebroadcastMsgs = [rebroadcastMsgs;rebroadcastMsg];    
    bundleIndex = bundleIndex + 1;
    
end %end of the for loop consisting of the sender

end