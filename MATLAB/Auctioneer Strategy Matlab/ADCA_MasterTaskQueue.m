classdef ADCA_MasterTaskQueue < handle
    
    properties (Access = private)
        TaskList = [];
    end
    
    properties (Access = public)
        m_TaskAssignmentMap = [];
    end
    
    methods
        function ADCA_MasterTaskQueue = ADCA_MasterTaskQueue()
            ADCA_MasterTaskQueue.TaskList = containers.Map();
            ADCA_MasterTaskQueue.m_TaskAssignmentMap = ADCA_AssignmentMap();
        end
        
        function [] = clearAndReset(obj)
            obj.TaskList = containers.Map();
            obj.m_TaskAssignmentMap = ADCA_AssignmentMap();
        end
        
        function [taskObj] = getTaskWithKey(obj, key)
            taskObj = [];
            
            keyExists = isKey(obj.TaskList,key.getTaskKeyString());
            if(keyExists)
                taskObj = obj.TaskList(key.getTaskKeyString());
            end
        end
        
        function appendToTaskList(obj, newTask)
            taskKey = newTask.m_TaskDescriptor.getTaskKey();
            obj.TaskList(taskKey.getTaskKeyString()) = newTask.copy(); %this will create a deep copy
        end
        
        function queueSize = sizeOfTaskList(obj)
            queueSize = size(obj.TaskList,1);
        end
        
        function [removedTask] = removeTaskAtKey(obj, key)
            removedTask = obj.TaskList(key.getTaskKeyString());
            remove(obj.TaskList, key.getTaskKeyString());
        end
        
        function [] = updateTaskAvailability(obj, taskKey, isAvailable)
            task = obj.getTaskWithKey(taskKey);
            task.m_TaskDescriptor.setTaskAvailability(isAvailable);
        end
        
        function [availableTasks] = getAvailableTasks(obj, agentID)
            availableTasks = ADCA_AvailableTaskQueue();
            mapKeys = obj.getTaskKeys();
            for m = 1:obj.sizeOfTaskList()
                currentTaskContainer = obj.getTaskWithKey(mapKeys(m));
                leadingAgent = currentTaskContainer.m_CurrentBids.LEADING_AGENT;
                isAvailable = currentTaskContainer.m_TaskDescriptor.AVAILABLE;
                if((isempty(leadingAgent) || (leadingAgent ~= agentID)) && (isAvailable))
                    availableTasks.appendTaskToList(obj.getTaskWithKey(mapKeys(m)));
                end
            end
        end
        
        function [prevLeaderID, lostTaskIDs] = awardAgentTask(obj,agentID,taskKey)
            lostTaskIDs = [];
            
            currentTask = obj.getTaskWithKey(taskKey);
            prevLeaderID = currentTask.updateLeadingAgent(agentID);
            
            obj.m_TaskAssignmentMap.agentAwardedTask(agentID,taskKey);
            
            %next we find all of the awards that had happened post the
            %task that was lost, remove and invalidate them.
            if(~isempty(prevLeaderID))
                lostTaskIDs = obj.agentLostTask(prevLeaderID,taskKey);
            end
        end
        
        function [lostTaskIDs] = agentLostTask(obj, agentID, taskID)
            lostTaskIDs = obj.m_TaskAssignmentMap.agentLostTask(agentID, taskID);
            if(isempty(lostTaskIDs))
                disp('Check here');
            end
            for idx = 1:numel(lostTaskIDs)
                obj.getTaskWithKey(lostTaskIDs(idx)).invalidateBid(agentID);
            end
        end
        
        
        function [taskKeys] = getTaskKeys(obj)
            stringKeys = keys(obj.TaskList);
            for i = 1:1:size(stringKeys,2)
                t = stringKeys(1,i);
                tSplit = strsplit(string(t),',');
                taskKeys(1,i) = ADCA_TaskKey(str2double(tSplit(1,1)),str2double(tSplit(1,2)),str2double(tSplit(1,3)));
            end
        end
        
        function [utility] = computeAgentUtility(obj, agentID)
            utility = 0;
            utilityMap = obj.computeCurrentUtility();
            if(isKey(utilityMap, num2str(agentID)))
                utility = utilityMap(num2str(agentID));
            end
        end
        
        function [utilityMap] = computeCurrentUtility(obj)
            utilityMap = containers.Map();
            for k = keys(obj.m_TaskAssignmentMap.m_AssignmentMap)
                theKey = k{1};
                utility = 0;

                if(isKey(obj.m_TaskAssignmentMap.m_AssignmentMap, theKey))
                    agentTaskBundle = obj.m_TaskAssignmentMap.m_AssignmentMap(theKey);
                    for valueIndex = 1:size(agentTaskBundle,2)
                        taskKey = agentTaskBundle(1,valueIndex);
                        utility = utility + obj.getTaskWithKey(taskKey).getAccompanyingBid(str2num(theKey)).UTILITY;
                    end
                end
                
                utilityMap(theKey) = utility;
            end
        end
        
        function [] = printTaskKeys(obj)
            currentKeys = getTaskKeys(obj);
        end
        
        function [] = printTaskLeaders(obj)
            currentKeys = getTaskKeys(obj);
            for m = 1:obj.sizeOfTaskList()
                leadingAgent = obj.getTaskWithKey(currentKeys(m)).m_CurrentBids.LEADING_AGENT;
                fprintf("The leading agent of Task: %s is %i. \n",char(currentKeys(m).getTaskKeyString()),leadingAgent);
            end
            fprintf("\n");
        end
        
        function[] = printCurrentAssignments(obj)
            obj.m_TaskAssignmentMap.print();
        end
    end
    
end