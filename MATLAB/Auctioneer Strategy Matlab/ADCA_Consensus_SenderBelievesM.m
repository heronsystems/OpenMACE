function [rebroadcastBid, invalidate] = ADCA_Consensus_SenderBelievesM(senderID, senderLeadingBid, receivingAgent, currentTime)

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

%The sender believes that the leading agent is neither the sender or the
%receiver
if((senderLeadingBid.AGENT_ID ~= senderID) &&...
        (senderLeadingBid.AGENT_ID ~= receiverID))
    
    if(isempty(receiverLeadingBid))
        ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
        %At this point the sending agents bid should be broadcasted to
        %all agents.
        rebroadcastBid = senderLeadingBid;

    elseif(receiverTask.m_CurrentBids.LEADING_AGENT == receiverID)
        %The reciever had initially believed that they had been the winner.
        %Need to compare the utility estimates to determine an actual
        %winner.
        if((senderLeadingBid.UTILITY - receiverLeadingBid.UTILITY) > tieEpsilon)
            %The receiver did indeed lose the task and the bid must be
            %better than the information available.
            ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
            %At this point the sending agents bid should be broadcasted to
            %all agents.
            rebroadcastBid = senderLeadingBid;
            %At this point the sending agents bid should be broadcasted to
            %all agents.
            
        elseif(abs(senderLeadingBid.UTILITY - receiverLeadingBid.UTILITY) <= tieEpsilon)
            %for now we will just assign to the individual with the smaller
            %ID.
            if(senderID < receiverID)
                ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
                %At this point the sending agents bid should be broadcasted to
                %all agents.
                rebroadcastBid = senderLeadingBid;
                %At this point the sending agents bid should be broadcasted to
                %all agents.
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
            %At this point the receiving agents bid should be broadcasted
            %to all agents.
            invalidate = true;
            rebroadcastBid = receiverLeadingBid;
        end
        
        %The reciever had initially believed that the sender had been the winner.
    elseif(receiverTask.m_CurrentBids.LEADING_AGENT == senderID)
        ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
        %At this point the sending agents bid should be broadcasted to
        %all agents.
        rebroadcastBid = senderLeadingBid;
        %The receiver has assigned a leader that is neither itself or sender.
        
    elseif(senderLeadingBid.AGENT_ID == receiverLeadingBid.AGENT_ID)
        if(senderLeadingBid.GENERATION_TIME > receiverLeadingBid.GENERATION_TIME)
            %The sending agents bid is more recent, update the bid
            receiverTask.receivedAdditionalBid(senderLeadingBid);
            %At this point the sending agents bid should be broadcasted to
            %all agents.
            rebroadcastBid = senderLeadingBid;
            
        elseif(senderLeadingBid.GENERATION_TIME < receiverLeadingBid.GENERATION_TIME)
            %The receiving agents bid is more recent, broadcast this
            rebroadcastBid = receiverLeadingBid;
        elseif(senderLeadingBid == receiverLeadingBid)
            %There is nothing to do as each agent has the same information
        end
        
        %The receiver believed the winner was not itself, the sender, or
        %the agent the sender believes the winner was.
    else
        if(((senderLeadingBid.UTILITY - receiverLeadingBid.UTILITY) > tieEpsilon) &&...
                (senderLeadingBid.GENERATION_TIME >= receiverLeadingBid.GENERATION_TIME))
            %The sender has more relevant information
            ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
            %At this point the sending agents bid should be broadcasted to
            %all agents.
            rebroadcastBid = senderLeadingBid;
            
        elseif((senderLeadingBid.UTILITY < receiverLeadingBid.UTILITY) &&...
                (senderLeadingBid.GENERATION_TIME < receiverLeadingBid.GENERATION_TIME))
            %The sender has a bid that does has less utility and was
            %generated before the senders bid...there is nothing to do
            %other than to distribute the senders leading bid.
            senderLeadingBid.updateValidity(false);
            receiverTask.receivedAdditionalBid(senderLeadingBid);
            %At this point the receiving agents bid should be broadcasted
            %to all agents.
            invalidate = true;
            rebroadcastBid = receiverLeadingBid;
            
        elseif((senderLeadingBid.UTILITY < receiverLeadingBid.UTILITY) &&...
                (senderLeadingBid.GENERATION_TIME > receiverLeadingBid.GENERATION_TIME))
            %The sender has a bid that has less utility but was generated
            %more recently. We shall award the task to the senders belief.
            ADCA_Consensus_AwardAgent(senderLeadingBid,receivingAgent);
            %At this point the sending agents bid should be broadcasted to
            %all agents.
            rebroadcastBid = senderLeadingBid;
            
        elseif((senderLeadingBid.UTILITY > receiverLeadingBid.UTILITY) &&...
                (senderLeadingBid.GENERATION_TIME < receiverLeadingBid.GENERATION_TIME))
            %Although the sending agent had a better utility, it still was
            %outdated. This could result from agents that may have dropped
            %out of the network. We will not assign a new winner and will
            %simply broadcast the receivers leading agent bid. 
            senderLeadingBid.updateValidity(false);
            receiverTask.receivedAdditionalBid(senderLeadingBid);
            %At this point the receiving agents bid should be broadcasted
            %to all agents.
            invalidate = true;
            rebroadcastBid = receiverLeadingBid;
        end
    end
    
end %end of if statement

end