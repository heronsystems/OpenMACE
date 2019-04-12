function [rebroadcastBid, invalidate] = ADCA_Consensus_SenderBelievesK(senderID, senderLeadingBid, receivingAgent, currentTime)

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
    %The receiver had no notion of a perceived leader
    ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
    rebroadcastBid = senderLeadingBid;
    %Distribute the sending agents bid to all agents.
    
    %The reciever had initially believed that they had been the winner.
elseif(receiverTask.m_CurrentBids.LEADING_AGENT == receiverID)
    %Need to compare the utility estimates to determine an actual
    %winner.
    if((senderLeadingBid.UTILITY - receiverLeadingBid.UTILITY) > tieEpsilon)
        %The receiver did indeed lose the task and the bid must be
        %better than the information available.
        ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
        %At this point the sending agents bid should be broadcasted to
        %all agents.
        rebroadcastBid = senderLeadingBid;
        
    elseif(abs(senderLeadingBid.UTILITY - receiverLeadingBid.UTILITY) <= tieEpsilon)
        %for now we will just assign to the individual with the smaller
        %ID.
        if(senderID < receiverID)
            ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
            %At this point the sending agents bid should be broadcasted to
            %all agents.
            rebroadcastBid = senderLeadingBid;
            
        else
            senderLeadingBid.updateValidity(false);
            receiverTask.receivedAdditionalBid(senderLeadingBid);
            %At this point the receiving agents bid should be broadcasted
            %to all agents.
            invalidate = true;
            rebroadcastBid = receiverLeadingBid;
            
        end
    elseif(senderLeadingBid.UTILITY < receiverLeadingBid.UTILITY)
        %The sender clearly hasn't received the information
        %that the receiving agent has, therefore, we only
        %need to invalidate this bid and then broadcast the
        %receivers leading bid to the other agents.
        senderLeadingBid.updateValidity(false);
        receiverTask.receivedAdditionalBid(senderLeadingBid);
        invalidate = true;
        rebroadcastBid = receiverLeadingBid;
    end
    
    %The reciever had initially believed that the sender had been the winner.
elseif(receiverTask.m_CurrentBids.LEADING_AGENT == senderID)
    if(receiverLeadingBid == senderLeadingBid)
        %The bids can be considered the same, therefore, we just need
        %to update the time associated with the bid. There is nothing
        %to do past this step.
        receiverLeadingBid.updateReceptionTime(currentTime);
    elseif((senderLeadingBid.GENERATION_TIME > receiverLeadingBid.GENERATION_TIME))
        %The time that is associated with this bid is more recent,
        %therefore, the receiver task queue needs to be updated.
        receiverTask.receivedAdditionalBid(senderLeadingBid);
        %The sending agent bid should be distributed to all agents.
        rebroadcastBid = senderLeadingBid;
        
    elseif((senderLeadingBid.GENERATION_TIME <= receiverLeadingBid.GENERATION_TIME))
        %This condition should not exist, the sender is distributing
        %old bid information and would seem not to be possible unless
        %propogating.
        disp('I do not understand how the sender could propogate an old bid.');
    end
    
    %The receiver has assigned a leader that is neither itself or sender.
elseif((receiverTask.m_CurrentBids.LEADING_AGENT ~= receiverID) &&...
        (receiverTask.m_CurrentBids.LEADING_AGENT ~= senderID))
    
    if((senderLeadingBid.UTILITY > receiverLeadingBid.UTILITY) &&...
            (senderLeadingBid.GENERATION_TIME >= receiverLeadingBid.GENERATION_TIME))
        %The sending agents bid is more recent and has more utility
        ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
        rebroadcastBid = senderLeadingBid;
        %Distribute the sending agents bid to all agents.
    elseif(abs(senderLeadingBid.UTILITY - receiverLeadingBid.UTILITY) <= tieEpsilon)
        %The sending agents bid is within tolerance to consider it a
        %tie.
        if(senderID < receiverLeadingBid.AGENT_ID)
            ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
            rebroadcastBid = senderLeadingBid;
            %At this point the sending agents bid should be broadcasted to
            %all agents.
        else
            senderLeadingBid.updateValidity(false);
            receiverTask.receivedAdditionalBid(senderLeadingBid);
            invalidate = true;
            rebroadcastBid = receiverLeadingBid;
            %At this point the receiving agents bid should be broadcasted
            %to all agents.
        end
    elseif((senderLeadingBid.UTILITY < receiverLeadingBid.UTILITY)...
            && (senderLeadingBid.GENERATION_TIME <= receiverLeadingBid.GENERATION_TIME))
        %The sending agents bid is not recent and doesn't have the
        %necessary utility to win.
        senderLeadingBid.updateValidity(false);
        receiverTask.receivedAdditionalBid(senderLeadingBid);
        invalidate = true;
        rebroadcastBid = receiverLeadingBid;
        %Propogate the receiver agents information.
        
    elseif((senderLeadingBid.UTILITY < receiverLeadingBid.UTILITY)...
            && (senderLeadingBid.GENERATION_TIME > receiverLeadingBid.GENERATION_TIME))
        %Even though the utility is less, it is more recent. Therefore,
        %the receiving agent must assume that the sending agent had
        %more knowledge to place the bid.
        ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
        rebroadcastBid = senderLeadingBid;
        
        %Distribute the sending agents bid to all agents.
    elseif((senderLeadingBid.UTILITY > receiverLeadingBid.UTILITY)...
            && (senderLeadingBid.GENERATION_TIME < receiverLeadingBid.GENERATION_TIME))
        %Ken Fix: In this case we should check and see if we have
        %already received the bid and have invalidated it.
        %The utility is greater, however, is not as recent.
        ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
        rebroadcastBid = senderLeadingBid;
        %Distribute the sending agents bid to all agents.
    end
end

end