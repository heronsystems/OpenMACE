function [] =  ADCA_RebroadcastBid(senderID, broadcastBid, currentTime, roboticSwarm, communicationTopology)

for receiverIndex = 1:size(roboticSwarm,1)
    receivingAgent = roboticSwarm(receiverIndex,1);
    receiverID = receivingAgent.m_RoboticParameters.AGENT_ID;
    
    %If the receiving agent cannot communicate with the sender, we
    %therefore break and go onto the next agent.
    if(communicationTopology(receiverID, senderID) ~= 1)
        continue;
    end
    
     [rebroadcastMsg] = receivingAgent.receiveBroadcastedBid(senderID, broadcastBid, currentTime);
     
     if(~isempty(rebroadcastMsg))
         %There was obviously something this agent needs to also propogate
         ADCA_RebroadcastBid(receiverID,rebroadcastMsg,currentTime,roboticSwarm,communicationTopology);
     end
end

end %end of function call ADCA_CommunicateBidToReceivers

