classdef GenericRoboticAgent < handle
    properties
        m_CurrentState = [];
        m_RoboticParameters = [];
        m_AuctionParameters = [];
        
        m_MasterTaskList = [];
        m_CurrentTaskQueue = [];
        m_CurrentBundle = [];
        m_CurrentExtendedBundle = [];
        
        biddingBundleFigure = [];
        utilityFigure = [];
        m_OperationalEnvironment = [];
    end
    
    methods
        function GenericRoboticAgent = GenericRoboticAgent(id, type, environment)
            GenericRoboticAgent.m_RoboticParameters = RoboticAgent_Params(id,type);
            GenericRoboticAgent.m_AuctionParameters = ADCA_AuctionParams();
            GenericRoboticAgent.m_CurrentState = roboticState();
            
            GenericRoboticAgent.m_MasterTaskList = ADCA_MasterTaskQueue();
            GenericRoboticAgent.m_CurrentTaskQueue = ADCA_CurrentTaskQueue();
            
            GenericRoboticAgent.m_CurrentBundle = ADCA_BiddingBundle();
            GenericRoboticAgent.m_OperationalEnvironment = environment;
            
            figureStringPrefix = 'Bidding Bundle Agent ';
            figureStringSuffix = num2str(id);
            
            GenericRoboticAgent.biddingBundleFigure = figure('Name', strcat(figureStringPrefix,figureStringSuffix));
            title('Current Iteration Bidding Bundle')
            xlabel('X Position (m)') % x-axis label
            ylabel('Y Position (m)') % y-axis label
            zlabel('Z Position (m)') % z-axis label
            
            %update the graph axis with the operational dimensions
            axis([environment.X_MIN, environment.X_MAX, environment.Y_MIN, environment.Y_MAX, environment.Z_MIN, environment.Z_MAX])
            
            grid on;
            hold on;
            
        end
        
        function [] = updateRoboticState(obj, state)
            obj.m_CurrentState = state;
        end
        
        function [] = clearCurrentBidBundle(obj)
            obj.m_CurrentBundle = ADCA_BiddingBundle();
            obj.m_CurrentExtendedBundle = [];
        end
        
        function [] = formulateExtendedBundleAssignment(obj)
            extendedBundle = ADCA_BiddingBundle();
            assignmentSize = obj.m_CurrentTaskQueue.sizeOfTaskList();
            for assignmentIndex = 1:1:assignmentSize
                currentTaskAssignment = obj.m_CurrentTaskQueue.getTaskAtIndex(assignmentIndex);
                extendedBundle.appendNewBid(currentTaskAssignment.getAssociatedBid());
            end
            obj.m_CurrentExtendedBundle = extendedBundle;
        end
        
        function [extendedBundle] = getExtendedBundleAssignment(obj)
            extendedBundle = obj.m_CurrentExtendedBundle;
        end
        
        function [state] = getConcludingState(obj)
            % GETCONCLUDINGSTATE Determine the expected state the agent
            % will be at the conclusion of the previous task or in its
            % current condition
            
            if(obj.m_CurrentTaskQueue.sizeOfTaskList() < 1)
                state = obj.m_CurrentState;
            else
                state = obj.m_CurrentTaskQueue.getLastTask().getEndingState();
            end
        end
        
        function [duration] = getExpectedTaskDuration(obj, task)
            % GETEXPECTEDTASKDURATION Given a ADCA_TASKCONTAINER, compute
            % the expected duration that this task may take based on the
            % host robotic agent.
            if(task.m_TaskDescriptor.getTaskType() == ADCA_TaskTypes.TASK_LOITER)
                duration = task.m_TaskDescriptor.LoiterDuration;
            elseif(task.m_TaskDescriptor.getTaskType() == ADCA_TaskTypes.TASK_SURVEY)
                duration = 1;
            end
        end
        
        function [] = newlyAvailableTasks(obj, newTasks)
            for i = 1:newTasks.sizeOfTaskList()
                obj.m_MasterTaskList.appendToTaskList(newTasks.getTaskAtIndex(i));
            end
        end
        
        function [currentTask, previousTaskKey] = updateHostTaskCompletion(obj)
            currentTask = [];
            previousTaskKey = [];
            previousTask = obj.m_CurrentTaskQueue.getCurrentTask();
            if(~isempty(previousTask))
                previousTaskKey = previousTask.m_TaskDescriptor.getTaskKey();
                currentTask = obj.updateHostTaskCompletion_Key(previousTaskKey); 
            end
        end
        
        function [currentTask] = updateHostTaskCompletion_Key(obj, taskKey)
            obj.m_CurrentTaskQueue.removeTaskAtKey(taskKey);
            currentTask = obj.m_CurrentTaskQueue.getCurrentTask();
        end
        
        function [] = updateTaskAvailability(obj, taskKey, isAvailable)
            obj.m_MasterTaskList.updateTaskAvailability(taskKey,isAvailable);
        end
        
        function [] = clearAndUpdateAvailableTasks(obj, newTasks)
            obj.m_MasterTaskList.clearAndReset();
            for i = 1:newTasks.sizeOfTaskList()
                obj.m_MasterTaskList.appendToTaskList(newTasks.getTaskAtIndex(i));
            end
        end
        
        function [availableTasks] = getAvailableTasks(obj)
            % GETAVAILABLETASKS Given the master task list and the current
            % active queue of the robotic agent, find tasks that do not
            % already exist within the active queue available for bidding.
            % This includes tasks that are not already marked as active by
            % other agents.
            availableTasks = obj.m_MasterTaskList.getAvailableTasks(obj.m_RoboticParameters.AGENT_ID);
        end
        
        function [] = updateHostBiddingBundle(obj, bidBundle)
            obj.m_CurrentBundle = bidBundle;
            %Go through the master task queue and do a greedy update
            for m = 1:obj.m_CurrentBundle.getBiddingBundleSize()
                bid = obj.m_CurrentBundle.getBidAtIndex(m);
                
                associatedTask = obj.m_MasterTaskList.getTaskWithKey(bid.m_TaskKey);
                associatedTask.receivedAdditionalBid(bid);
                obj.m_MasterTaskList.awardAgentTask(bid.AGENT_ID,bid.m_TaskKey);
                
                %All this is doing is mapping the type to something we can
                %insert into the current task queue
                assignmentTask = ADCA_TaskAssignment();
                assignmentTask.updateFromBundle(associatedTask.m_TaskDescriptor,bid);
                obj.m_CurrentTaskQueue.appendTaskToList(assignmentTask);
            end
            obj.formulateExtendedBundleAssignment();
        end
        
        function [rebroadcastMsgs] = receivedCompetingBundle(obj, senderID, bidBundle, currentTime)
            [rebroadcastMsgs, ~] = ADCA_EstablishBundleConsensus(senderID, bidBundle, obj, currentTime);
        end
        
        function [rebroadcastMsg] = receiveBroadcastedBid(obj, senderID, broadcastBid, currentTime)
            rebroadcastMsg = [];
%             [rebroadcastMsg, ~] = ADCA_EvaluateRules(senderID,broadcastBid,obj,currentTime);    
        end
                
        function [] = graphCurrentWorkload(obj)
            
        end
        
        function [] = graphCurrentUtility(obj)
            
        end
        
        function [plottingPath] = graphCurrentTaskPath(obj)
            figureStringPrefix = 'Task Path For Agent ';
            figureStringSuffix = num2str(obj.m_RoboticParameters.AGENT_ID);
            
            taskPath = figure('Name', strcat(figureStringPrefix,figureStringSuffix));
            
            title('Current Task Path')
            xlabel('X Position (m)') % x-axis label
            ylabel('Y Position (m)') % y-axis label
            zlabel('Z Position (m)') % z-axis label
            
            %update the graph axis with the operational dimensions
            axis([obj.m_OperationalEnvironment.X_MIN, obj.m_OperationalEnvironment.X_MAX, obj.m_OperationalEnvironment.Y_MIN, obj.m_OperationalEnvironment.Y_MAX, obj.m_OperationalEnvironment.Z_MIN, obj.m_OperationalEnvironment.Z_MAX])
            
            grid on;
            hold on;
            
            agentPosition = obj.m_CurrentState.POSITION;
%             plot3(taskPath.CurrentAxes,agentPosition.x,agentPosition.y,agentPosition.z,'o');
            
            plottingPath = obj.m_CurrentTaskQueue.graphCurrentTaskQueue(taskPath.CurrentAxes, agentPosition);
        end
        
        function plotCurrentAgentSchedule(obj,axes)
            obj.m_CurrentTaskQueue.plotCurrentSchedule(axes);
        end
    end
end