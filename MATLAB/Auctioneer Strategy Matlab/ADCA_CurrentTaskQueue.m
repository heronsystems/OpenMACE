classdef ADCA_CurrentTaskQueue < ADCA_AbstractGenericList
    
    methods
        function [obj] = createEmptyList(obj)
            if isempty(obj.List)
                obj.List = ADCA_TaskAssignment();
            end
        end
        
        function [taskObject] = getCurrentTask(obj)
            taskObject = obj.getTaskAtIndex(1);
        end
        
        function [taskObject] = getTaskAtIndex(obj,index)
            taskObject = obj.getItemAtIndex(index);
        end
        
        function [taskObject] = getLastTask(obj)
            taskObject = obj.getItemAtIndex(obj.sizeOfTaskList());
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
        
        function [] = removeTaskAtKey(obj, removeKeys)
            for idxKeys = 1:numel(removeKeys)
                for idxList = 1:obj.sizeOfList()
                    if(obj.getTaskAtIndex(idxList).m_TaskDescriptor.getTaskKey() == removeKeys(idxKeys))
                        obj.removeTaskAtIndex(idxList);
                        break;
                    end
                end
            end
        end
        
        function [taskList] = getCurrentTaskList(obj)
            taskList = obj.getCurrentList;
        end
        
        function [] = appendTaskBundle(obj, masterQueue, currentBundle)
            for i = 1:currentBundle.getBiddingBundleSize()
                currentBid = currentBundle.getBidFromBundleAt(i);
                obj.appendTaskToList(masterQueue.getTask(currentBid.TASK_KEY));
            end
        end
        
        function [] = printCurrentTaskQueue(obj)
            for i = 1:obj.sizeOfTaskList()
                currentTask = obj.getTaskAtIndex(i);
                currentTask.m_TaskDescriptor.getTaskKey().print();

%                 currentTask.m_TaskDescriptor.print();
            end
            disp('\n');
        end
        
        % Sheng: add this function to get a list of tasks 
        function [taskLocation] = getCurrentTaskQueueLocation(obj)
            taskLocation = [];
            for i = 1:obj.sizeOfTaskList()
                currentTask = obj.getTaskAtIndex(i);
                taskLocation = [taskLocation; [currentTask.m_TaskDescriptor.LoiterLocation.x currentTask.m_TaskDescriptor.LoiterLocation.y]];
            end
        end
        
        function [position] = graphCurrentTaskQueue(obj, plottingAxis, startingPosition)
            position = zeros(obj.sizeOfTaskList() + 1, 3);
            position(1,:) = startingPosition.getPositionVector()';
            plotIndex = 2;
            for i = 1:obj.sizeOfTaskList()
                currentTask = obj.getTaskAtIndex(i);
                positionStart = currentTask.m_BeginningState.POSITION.getPositionVector()';
                positionEnd = currentTask.m_EndingState.POSITION.getPositionVector()';
                
                position(plotIndex,:) = positionStart;
                position(plotIndex+1,:) = positionEnd;
                
                text(position(plotIndex,1),position(plotIndex,2),position(plotIndex,3), ['T' num2str(currentTask.m_TaskDescriptor.getTaskID())], 'Parent', plottingAxis);
                plotIndex = plotIndex + 2;
            end
            plot3(plottingAxis, position(:,1),position(:,2),position(:,3),'r-');
            plot3(plottingAxis, position(2:end,1),position(2:end,2),position(2:end,3),'r+');
        end
        
        function [] = plotCurrentSchedule(obj, plottingAxis)
            for i = 1:obj.sizeOfTaskList()
                currentTask = obj.getTaskAtIndex(i);
                plot(plottingAxis, [currentTask.m_BeginningState.TIME currentTask.m_EndingState.TIME],[1 1],'-', 'Linewidth',10);
            end
            set(plottingAxis,'YTick',[]);
            set(plottingAxis,'YTicklabel',[]);
        end
    end
end