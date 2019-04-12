classdef ADCA_BidDescriptor < matlab.mixin.Copyable
    % someClass Summary of this class goes here
    % Detailed explanation goes here
    properties
        AGENT_ID = []; %item to be transmitted reflecting the current agent who generated the bid
        m_TaskKey = []; %item to be transmitted reflecting the key related to the task the bid is pertinent to
        
        % WORK - Item to be transmitted relating to the assumed workload
        % that an agent is going to do in order to perform the task. This
        % value is independently tracked from cost as their may be imposed
        % additional penaltities that influence the cost different from
        % work.
        WORK = 0.0;
        
        % COST - Item to be transmitted relating to the cost of the agent
        % performing the task. The cost is calculated based on a number of
        % parameters that may be independently defined for each type of
        % task.
        COST = inf;
        REWARD = -inf;
        UTILITY = -inf;
        GENERATION_TIME = 0; %item to be transmitted reflecting the current agent who generated the bid
        UPDATE_TIME = 0;
        VALIDITY = true;
        CLEARING_BID = false;
        WARPED_UTILITY = 0;
        %Properties relating to execution of the task not needed to be
        %transmitted between agents
        m_TransitionState = [];
        m_BeginningState = [];
        m_EndingState = [];
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            % Make a deep copy of all of the objects
            cpObj.WORK = copy(obj.AGENT_ID);
            cpObj.m_TaskKey = copy(obj.m_TaskKey);
            cpObj.WORK = copy(obj.WORK);
            cpObj.COST = copy(obj.COST);
            cpObj.REWARD = copy(obj.REWARD);
            cpObj.UTILITY = copy(obj.UTILITY);
            cpObj.GENERATION_TIME = copy(obj.GENERATION_TIME);
            cpObj.UPDATE_TIME = copy(obj.UPDATE_TIME);
            cpObj.VALIDITY = obj.VALIDITY;
            cpObj.CLEARING_BID = obj.CLEARING_BID;
            cpObj.WARPED_BID = obj.WARPED_BID;
            
            cpObj.m_TransitionState = copy(obj.m_TransitionState);
            cpObj.m_BeginningState = copy(obj.m_BeginningState);
            cpObj.m_EndingState = copy(obj.m_EndingState);
        end
    end
    methods
        function ADCA_BidDescriptor = ADCA_BidDescriptor()
            ADCA_BidDescriptor.GENERATION_TIME = clock;
            ADCA_BidDescriptor.UPDATE_TIME = ADCA_BidDescriptor.GENERATION_TIME;

        end
        
        function [] = useAsClearingBid(obj, shouldClear)
            obj.CLEARING_BID = shouldClear;
        end
        
        function [] = updateBidProperties(obj, id, key, work, cost, reward, time)
            obj.AGENT_ID = id;
            obj.m_TaskKey = key;
            obj.WORK = work;
            obj.COST = cost;
            obj.REWARD = reward;
            obj.GENERATION_TIME = time;
            obj.UPDATE_TIME = time;
            obj.estimateUtility();
        end
        
        function [] = updateStateProperties(obj, transitionState, begState, endState)
            obj.m_TransitionState = copy(transitionState);
            obj.m_BeginningState = copy(begState);
            obj.m_EndingState = copy(endState);
        end
        
        function [] = estimateUtility(obj)
            %estimateUtility update the utility value of the bid
            %   Function that is automatically called when creating a
            %   ADCA_BidDescriptor object to determine the appropriate
            %   utility.
            obj.UTILITY = obj.REWARD - obj.COST;
        end
        
        function [] = updateReceptionTime(obj, time)
            obj.UPDATE_TIME = time;
        end
        
        function [leader] = isWarpedBidGreater(obj, competingBid)
           if((isempty(competingBid)) || (obj.WARPED_UTILITY > competingBid.WARPED_UTILITY))
               leader = true;
           else
               leader = false;
           end  
        end
            
        function [leader] = isBidGreater(obj, competingBid)
            %isBidGreater evaluate if current bid has more utility
            %   Function that returns true if this current bid has a
            %   greater utility than the competing bid argument. Will also
            %   return true if the competing bid is empty.
            if((isempty(competingBid)) || (obj.UTILITY > competingBid.UTILITY))
                leader = true;
            else
                leader = false;
            end
        end
        
        function [] = updateValidity(obj, valid)
            obj.VALIDITY = valid;
        end
        
        function [] = print(obj)
            obj.m_TaskKey.print();
            fprintf('\n');
            disp(obj);
            fprintf('Transition State \n'); obj.m_TransitionState.print();
            fprintf('Beginning State \n'); obj.m_BeginningState.print();
            fprintf('Ending State \n'); obj.m_EndingState.print();
        end
    end
    
    methods
        function tf = eq(obj1,obj2)
            tf = true;
            if (obj1.AGENT_ID ~= obj2.AGENT_ID)
                tf = false;
                return;
            elseif (obj1.m_TaskKey ~= obj2.m_TaskKey)
                tf = false;
                return;
            elseif (obj1.WORK ~= obj2.WORK)
                tf = false;
                return;
            elseif (obj1.COST ~= obj2.COST)
                tf = false;
                return;
            elseif (obj1.REWARD ~= obj2.REWARD)
                tf = false;
                return;
            elseif (obj1.UTILITY ~= obj2.UTILITY)
                tf = false;
                return;
            elseif (obj1.GENERATION_TIME ~= obj2.GENERATION_TIME)
                tf = false;
                return;
            end
        end
    end
    
end
