classdef ADCA_TaskAssignment < matlab.mixin.Copyable
    
    properties
        m_TaskDescriptor %The current description of the task
        TaskDistance = 0.0;
        m_AccompanyingBid = [];
        
        m_TransitionState = [];
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
            cpObj.m_AccompanyingBid = copy(obj.m_AccompanyingBid);
            
            cpObj.m_TransitionState = copy(obj.m_TransitionState);
            cpObj.m_BeginningState = copy(obj.m_BeginningState);
            cpObj.m_EndingState = copy(obj.m_EndingState);
        end
    end
    
    methods
        
        function ADCA_TaskAssignment = ADCA_TaskAssignment()
            
        end
        
        function [] = updateFromBundle(obj, descriptor, bid)
            obj.setTaskDescriptor(descriptor);
            obj.setAssociatedBid(bid);
            obj.updateStateProperties(bid.m_TransitionState, bid.m_BeginningState, bid.m_EndingState);
        end
        
        function [cost, work] = getBidPenalties(obj)
            cost = obj.m_AccompanyingBid.COST;
            work = obj.m_AccompanyingBid.WORK;
        end
        
        function [utility] = getEstimatedBidUtility(obj)
            utility = obj.m_AccompanyingBid.UTILITY;
        end
        
        function [] = updateStateProperties(obj, transitionState, begState, endState)
            obj.m_TransitionState = copy(transitionState);
            obj.m_BeginningState = copy(begState);
            obj.m_EndingState = copy(endState);
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
        
        function [] = setAssociatedBid(obj, bid)
            obj.m_AccompanyingBid = bid;
        end
        
        function [associatedBid] = getAssociatedBid(obj)
            associatedBid = obj.m_AccompanyingBid;
        end
    end
    
end
