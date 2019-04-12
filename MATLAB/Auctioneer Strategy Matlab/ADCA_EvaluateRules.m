function [rebroadcastMsg, invalidate]  = ADCA_EvaluateRules(senderID, senderLeadingBid, receivingAgent, currentTime)

receiverID = receivingAgent.m_RoboticParameters.AGENT_ID;

if(senderLeadingBid.CLEARING_BID)
    [rebroadcastMsg, invalidate] = ADCA_Consensus_SenderBelievesNone(senderID, senderLeadingBid, receivingAgent, currentTime);
    %Initial checks are assessing the validity that the sending
    %agent had won the task.
elseif(senderLeadingBid.AGENT_ID == senderID)
    [rebroadcastMsg, invalidate] = ADCA_Consensus_SenderBelievesK(senderID, senderLeadingBid, receivingAgent, currentTime);
    
    %Secondary checks are checking the validity in the receivers
    %perception that the receieving agent was actually the winner.
elseif(senderLeadingBid.AGENT_ID == receiverID)
    [rebroadcastMsg, invalidate] = ADCA_Consensus_SenderBelievesI(senderID, senderLeadingBid, receivingAgent, currentTime);
    
    %The sending agent believes that neither the itself or the
    %receiver are winners for this task.
elseif((senderLeadingBid.AGENT_ID ~= senderID)...
        && (senderLeadingBid.AGENT_ID ~= receiverID))
    [rebroadcastMsg, invalidate] = ADCA_Consensus_SenderBelievesM(senderID, senderLeadingBid, receivingAgent, currentTime);
end

end

