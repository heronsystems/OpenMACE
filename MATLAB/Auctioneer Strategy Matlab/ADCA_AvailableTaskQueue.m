classdef ADCA_AvailableTaskQueue < ADCA_AbstractGenericList
    
    methods
        function [obj] = createEmptyList(obj)
            if isempty(obj.List)
                obj.List = ADCA_TaskContainer();
            end
        end
        
        function [taskObject] = getTaskAtIndex(obj,index)
            taskObject = obj.getItemAtIndex(index);
        end
        
        function [] = appendTaskToList(obj, newTask)
            obj.appendToList(newTask);
        end
        
        function listSize = sizeOfTaskList(obj)
            listSize = obj.sizeOfList();
        end
        
        function [task] = removeTaskAtIndex(obj, index)
            task = obj.removeAtIndex(index);
        end
        
        function [taskList] = getCurrentTaskList(obj)
            taskList = obj.getCurrentList;
        end
        
        % Sheng: add this function to quickly find the ID's of current
        % tasks
        function [taskID]  = getCurrentTaskID(obj)
            taskID = [];
            taskList = obj.getCurrentTaskList;
            for k = 1:length(taskList)
                taskID = [taskID;taskList(k).m_TaskDescriptor.getTaskID];
            end
        end
        
        function [] = printTaskList(obj)
            for i = 1:1:obj.sizeOfList()
                obj.getItemAtIndex(i).m_TaskDescriptor.LoiterLocation.printLocation();
            end
        end
    end
    
end
