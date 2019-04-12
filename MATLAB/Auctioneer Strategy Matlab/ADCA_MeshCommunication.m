function [] = ADCA_MeshCommunication(roboticAgents, communicationTopology)

for k = 1:size(roboticAgents,1)
    sendingAgent = roboticAgents(k,1);
    senderID = sendingAgent.m_RoboticParameters.AGENT_ID;
    sendingBundle = sendingAgent.m_CurrentBundle;
    
    if(sendingBundle.getBiddingBundleSize() <= 0)
        %this agent has no new bundle to transmit
        continue;
    end
    
    %Ken Note: This deviates from the convention of just transmitting the
    %bundle by itself, however, this is required per any mitigation methods
    %for vehicles becoming out of sync with eachother.
    sendingBundle = sendingAgent.getExtendedBundleAssignment();
    
    %This is actually a shortcut in the event an agent lost everything, as
    %we know that we are currently transmitting the entire queue we can
    %stop here, the original bundle will have been still full, however
    %since it has been cleared a secondary check here is important
    if(sendingBundle.getBiddingBundleSize() <= 0)
        %If we got this far this means the agent had lost everything
        continue;
    end
    
    %The inner loop will consist of loop through the entire network of
    %agents and determining who would be in receipt of the bid and
    %participate in the auction.
    
    %During each loop we only are going to update the receivers information
    %as they are the only ones with the newly perceived knowledge
    for j = 1:size(roboticAgents,1)
        currentTime = datetime('now');
        
        receivingAgent = roboticAgents(j,1);
        receiverID = receivingAgent.m_RoboticParameters.AGENT_ID;
        %If the receiving agent cannot communicate with the sender, we
        %therefore break and go onto the next agent.
        if(communicationTopology(j,k) ~= 1)
            continue;
        end
        
        [rebroadcastMsgs] = receivingAgent.receivedCompetingBundle(senderID, sendingBundle, currentTime);
        
        if(~isempty(rebroadcastMsgs))
            for msgIndex = 1:1:size(rebroadcastMsgs,1)
                ADCA_RebroadcastBid(receiverID, rebroadcastMsgs(msgIndex,1), currentTime, roboticAgents, communicationTopology);
            end
        end
    end %end of the for loop consisting of the sender
end