classdef ADCA_AssignmentMap < matlab.mixin.Copyable
    
    %Key is the agent
    %Values in the map are the list of assignments
    properties
        m_AssignmentMap = []; %containers.Map: Object containing the perceived assignments
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            if(~isempty(obj.m_AssignmentMap))
                %copying maps is strange in matlab
                cpObj.m_AssignmentMap = containers.Map(obj.m_AssignmentMap.keys,obj.m_AssignmentMap.values);
            end
        end
    end
    
    methods
        function ADCA_AssignmentMap = ADCA_AssignmentMap()
            ADCA_AssignmentMap.m_AssignmentMap = containers.Map();
        end
        
        function [] = agentAwardedTask(obj, agentID, taskID)
            agentString = num2str(agentID);
            if(~isKey(obj.m_AssignmentMap, agentString))
                obj.m_AssignmentMap(agentString) = taskID;
            else
                %insertIndex = size(obj.m_AssignmentMap(agentString),2) + 1;
                obj.m_AssignmentMap(agentString) = [obj.m_AssignmentMap(agentString), taskID];
            end
        end
        
        function [removeKeys] = agentLostTask(obj, agentID, taskKey)
            removeKeys = [];
            agentString = num2str(agentID);
            removeColumn = 0;
            agentTaskBundle = obj.m_AssignmentMap(agentString);
            for i = 1:size(agentTaskBundle,2)
                if(taskKey == agentTaskBundle(:,i))
                    removeColumn = i;
                    break;
                end
            end
            if(removeColumn ~= 0)
                removeKeys = agentTaskBundle(1,removeColumn:end);
                if(removeColumn == 1)
                    obj.m_AssignmentMap(agentString) = [];
                else
                    obj.m_AssignmentMap(agentString) = agentTaskBundle(1:removeColumn-1);
                end
            end
            
            if(isempty(removeKeys))
                disp('Check Here');
            end
        end
        
        function [] = print(obj)
            for k = keys(obj.m_AssignmentMap)
                theKey = k{1};
                fprintf('Agent ID: %s',theKey);
                fprintf(" ");
                agentTaskBundle = obj.m_AssignmentMap(theKey);
                for valueIndex = 1:size(agentTaskBundle,2)
                    agentTaskBundle(1,valueIndex).print();
                    fprintf(" ");
                end
            end
            fprintf("\n");
        end
    end
    
end