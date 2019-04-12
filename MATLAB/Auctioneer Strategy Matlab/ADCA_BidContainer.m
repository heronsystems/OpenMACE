classdef ADCA_BidContainer < matlab.mixin.Copyable
    
    properties
        m_BidDescriptors = []; %containers.Map: Object containing all recieved bids of the host agent for the task
        LEADING_AGENT; %double: scalar value containing the ID of the agent having the current winning bid for the task
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            % Make a deep copy of all of the objects
            cpObj = ADCA_BidContainer();
            
            if(~isempty(obj.m_BidDescriptors))
                %copying maps is strange in matlab
                cpObj.m_BidDescriptors = containers.Map(obj.m_BidDescriptors.keys,obj.m_BidDescriptors.values);
            end
            
            cpObj.LEADING_AGENT = obj.LEADING_AGENT;
        end
    end
    
    methods
        function ADCA_BidContainer = ADCA_BidContainer()
            ADCA_BidContainer.m_BidDescriptors = containers.Map();
            ADCA_BidContainer.LEADING_AGENT = [];
        end
        
        function [leadingBid] = getLeadingBid(obj)
            leadingBid = [];
            if(~isempty(obj.LEADING_AGENT))
                leadingBid = obj.m_BidDescriptors(num2str(obj.LEADING_AGENT));
            end            
        end
        
        function [leadingTime] = getLatestUpdate(obj)
            leadingTime = [];
            for k = keys(obj.m_BidDescriptors)
                thekey = k{1};
                if(isempty(leadingTime) || any(obj.m_BidDescriptors(thekey).TIME > leadingTime))
                    leadingTime = obj.m_BidDescriptors(thekey).TIME;
                end
            end
        end
        
        function [] = invalidateAgentBid(obj, agentID)
            if(agentID == obj.LEADING_AGENT)
               obj.LEADING_AGENT = []; 
            end
            
            bid = obj.m_BidDescriptors(num2str(agentID));
            bid.updateValidity(false);
        end
        
        function [prevLeader] = setLeadingAgent(obj, agentID)
            prevLeader = obj.LEADING_AGENT;
            obj.LEADING_AGENT = agentID;
        end
        
        function [competitive] = receivedBid(obj, bid)
            %in here we would have some sort of handling in the event there
            %is a discrepency, for now we will proceed assuming perfect
            %knowledge
            obj.m_BidDescriptors(num2str(bid.AGENT_ID)) = bid;
            competitive = obj.wouldBidLead(bid);
        end
        
        function [bid] = getAgentBid(obj, agentID)
            if(isKey(obj.m_BidDescriptors,num2str(agentID)))
                bid = obj.m_BidDescriptors(num2str(agentID));
            else
                bid = [];
            end
        end

        
        function [lead] = wouldBidLead(obj, bid)
            % ISBIDCOMPETITIVE A function evaluating whether a current bid
            % is competitive to the bids this container is currently aware
            % of.
            
            %Let us assume the worst case that the bid is not competitive.
            lead = false;
            
            %if we have not established a current leading agent, then this
            %current bid is currently the most competitive option we are
            %aware of.
            if(isempty(obj.LEADING_AGENT))
                lead = true;
                return
            end
            
            %this means there is a current leader established in this
            %object, let us grab the currently leading bid descriptor
            leadingBidDescriptor = obj.m_BidDescriptors(num2str(obj.LEADING_AGENT));
            
            %evaluate whether or not the current bid is greater than the
            %current supposed leader
            if(bid.isBidGreater(leadingBidDescriptor))
                lead = true;
            else
                lead = false;
            end
        end
        
    end
    
end
