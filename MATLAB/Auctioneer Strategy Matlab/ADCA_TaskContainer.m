classdef ADCA_TaskContainer < matlab.mixin.Copyable
    
    properties
        m_TaskDescriptor %The current description of the task
        TaskDistance = 0.0;
        m_CurrentBids = []; %The currently received bids for the host agent in the form of BidContainer
        
        %The following objects are only kept here in regards to the host
        %agent and are never updated by any other agent. It may be better
        %to eventually store them in a seperate object.
        m_BeginningState = [];
        m_EndingState = [];
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            cpObj = ADCA_TaskContainer();
            % Make a deep copy of all of the objects
            cpObj.m_TaskDescriptor = copy(obj.m_TaskDescriptor);
            cpObj.TaskDistance = obj.TaskDistance;
            cpObj.m_CurrentBids = copy(obj.m_CurrentBids);
            cpObj.m_BeginningState = copy(obj.m_BeginningState);
            cpObj.m_EndingState = copy(obj.m_EndingState);
        end
    end
    
    methods
        function ADCA_TaskContainer = ADCA_TaskContainer()
            ADCA_TaskContainer.m_CurrentBids = ADCA_BidContainer();
            ADCA_TaskContainer.m_BeginningState = roboticState();
            ADCA_TaskContainer.m_EndingState = roboticState();
        end
        
        function [] = setTaskDescriptor(obj, descriptor)
            obj.m_TaskDescriptor = descriptor;
        end
        
        function [] = setBeginningState(obj, state)
            obj.BeginningState = state;
        end
        
        function [] = setEndingState(obj, state)
            obj.EndingState = state;
        end
        
        function duration = getExpectedDuration(obj)
            duration = 0;
        end
        
        function state = getBeginningState(obj)
            state = obj.m_BeginningState;
        end
        
        function state = getEndingState(obj)
            state = obj.m_EndingState;
        end
        
        function [competitive] = isBidCompetitive(obj, bid)
            competitive = obj.m_CurrentBids.wouldBidLead(bid);
        end
        
        function [competitive] = isWarpedBidCompetitive(obj, bid)
            competitive = obj.m_CurrentBids.wouldBidLead(bid);
        end
        
        function [] = receivedAdditionalBid(obj, bid)
                obj.m_CurrentBids.receivedBid(bid);
        end
        
        function [leadingBid] = whatIsLeadingBid(obj)
            leadingBid = obj.m_CurrentBids.getLeadingBid();
        end
        
        function [prevLeader] = updateLeadingAgent(obj, agentID)
            prevLeader = obj.m_CurrentBids.setLeadingAgent(agentID);
        end
        
        function [] = invalidateLeadingAgent(obj)
            obj.m_CurrentBids.invalidateAgentBid(obj.m_CurrentBids.LEADING_AGENT);
        end
        
        function [] = invalidateBid(obj, agentID)
            obj.m_CurrentBids.invalidateAgentBid(agentID);
        end
        
        function [bid] = getAccompanyingBid(obj, agentID)
            bid = obj.m_CurrentBids.getAgentBid(agentID);
        end
    end
    
end
