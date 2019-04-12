classdef ADCA_TaskDescriptor < matlab.mixin.Copyable
    
    properties(Access = protected)
        m_TaskKey = [];
    end
    
    properties
        BaseReward = 100;
        Time_Created = 0; %The time the task was created
        Time_Penalty = 0;
        Time_RequiredStart = 0; %The time the task is required to start by
        Time_RequiredEnd = 0; %The time the task is required to be completed by
        AVAILABLE = true;
    end %end of properties statement
    
    methods (Abstract)
        graphTaskProperties(obj, axes, cmap);
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            % Setup the base type copy constructor
            cpObj = copyElement@matlab.mixin.Copyable(obj);
            
            % Make a deep copy of all of the objects
            cpObj.m_TaskKey = copy(obj.m_TaskKey);
            cpObj.BaseReward = obj.BaseReward;
            cpObj.Time_Created = obj.Time_Created;
            cpObj.Time_Penalty = obj.Time_Penalty;
            cpObj.Time_RequiredStart = obj.Time_RequiredStart;
            cpObj.Time_RequiredEnd = obj.Time_RequiredEnd;
            cpObj.AVAILABLE = obj.AVAILABLE;            
        end
    end
    
    methods
        function ADCA_TaskDescriptor = ADCA_TaskDescriptor(creatorID, taskID, taskType)
            % ADCA_TaskContainer A container object holding the pertinent
            % details of a given task.
            ADCA_TaskDescriptor.m_TaskKey = ADCA_TaskKey(creatorID,taskID,taskType);
        end
        
        function TaskKey = getTaskKey(obj)
            % GETTASKKEY A function to get the uniquely identifying key
            % from this task object.
%             keyCreator = num2str(obj.m_TaskKey.CREATOR_ID);
%             keyID = num2str(obj.m_TaskKey.TASK_ID);
%             keyType = num2str(obj.m_TaskKey.TASK_TYPE);
%             keySpace = ',';
%             TaskKey = strcat(keyCreator,keySpace,keyID,keySpace,keyType);
            TaskKey = obj.m_TaskKey;
        end
        
        function taskType = getTaskType(obj)
            taskType = obj.m_TaskKey.TASK_TYPE;
        end
        
        function taskID = getTaskID(obj)
            taskID = obj.m_TaskKey.TASK_ID;
        end
        
        function [isAble]=canAgentAddressTask(obj, agentParams)
            % CANAGENTADDRESSTASK A function to assess whether given the
            % current properties of a robotic agent, can they successfully
            % address the given task.
            isAble = agentParams.CompatibilityMatrix(1,uint32(obj.m_TaskKey.TASK_TYPE));
        end
        
        function [] = setTaskAvailability(obj, isAvailable)
            obj.AVAILABLE = isAvailable;
        end
        
        function [] = print(obj)
            disp(obj);
        end
        
        
    end %end of methods statement
end
