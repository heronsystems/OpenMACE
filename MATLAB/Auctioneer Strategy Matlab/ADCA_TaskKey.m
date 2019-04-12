classdef ADCA_TaskKey < matlab.mixin.Copyable
    %ADCA_TASKKEY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        CREATOR_ID = 0; %A unique identifor indicated who was the creator of the task
        TASK_ID = 0; %A unique identifier for the task
        TASK_TYPE; %The type of task this is
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            % Setup the base type copy constructor
            cpObj = copyElement@matlab.mixin.Copyable(obj);
            
            % Make a deep copy of all of the objects
            cpObj.CREATOR_ID = obj.CREATOR_ID;
            cpObj.TASK_ID = obj.TASK_ID;
            cpObj.TASK_TYPE = obj.TASK_TYPE;
        end
    end
    
    methods
        function ADCA_TaskKey = ADCA_TaskKey(creatorID, taskID, taskType)
            %ADCA_TASKKEY Construct an instance of this class
            %   Detailed explanation goes here
            ADCA_TaskKey.CREATOR_ID = creatorID;
            ADCA_TaskKey.TASK_ID = taskID;
            ADCA_TaskKey.TASK_TYPE = taskType;
        end
        
        function TaskKey = getTaskKeyString(obj)
            % GETTASKKEY A function to get the uniquely identifying key
            % from this task object.
            keyCreator = num2str(obj.CREATOR_ID);
            keyID = num2str(obj.TASK_ID);
            keyType = num2str(obj.TASK_TYPE);
            keySpace = ',';
            TaskKey = strcat(keyCreator,keySpace,keyID,keySpace,keyType);
        end
        
       function [] = print(obj)
            fprintf('Task Key %i %i %i',obj.CREATOR_ID,obj.TASK_ID,obj.TASK_TYPE);
        end
    end
    
    methods
        function tf = eq(obj1,obj2)
            if (obj1.CREATOR_ID ~= obj2.CREATOR_ID)
                tf = false;
                return;
            end
            if (obj1.TASK_ID ~= obj2.TASK_ID)
                tf = false;
                return;
            end
            if (obj1.TASK_TYPE ~= obj2.TASK_TYPE)
                tf = false;
                return;
            end
            tf = true;
        end
    end
    
end
    
