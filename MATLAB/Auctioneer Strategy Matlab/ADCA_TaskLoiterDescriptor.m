classdef ADCA_TaskLoiterDescriptor < ADCA_TaskDescriptor
    
    properties
        LoiterLocation = []; %Location object that the vehicle should target its loiter pattern
        LoiterDuration = 10; %The duration is ms that the vehicle is required to loiter at the given target location
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            
            cpObj = copyElement@ADCA_TaskDescriptor(obj);
            
            % Make a deep copy of all of the objects
            cpObj.LoiterLocation = copy(obj.LoiterLocation);
            cpObj.LoiterDuration = obj.LoiterDuration;
        end
    end
    
    methods
        function ADCA_TaskLoiterDescriptor = ADCA_TaskLoiterDescriptor(generatingID, taskID)
            
            % ADCA_TASKLOITERDESCRIPTOR Object inheriting from
            % ADCA_TASKDESCRIPTOR that details the necessary elements
            % describing a loiter task.
            args{1} = generatingID;
            args{2} = taskID;
            args{3} = ADCA_TaskTypes.TASK_LOITER;
            
            % Call superclass constructor before accessing object
            ADCA_TaskLoiterDescriptor = ADCA_TaskLoiterDescriptor@ADCA_TaskDescriptor(args{:});
            ADCA_TaskLoiterDescriptor.LoiterLocation = location();
%             ADCA_TaskLoiterDescriptor.LoiterDuration =  (100-1)*rand() + 1;
            ADCA_TaskLoiterDescriptor.LoiterDuration =  50; % Sheng: fix loiter task duration
        end
        
        function setLoiterDuration(obj, duration)
            % SETLOITERDURATION function updating the required loiter
            % duration for this task.
            obj.LoiterDuration = duration;
        end
        
        function setLoiterLocation(obj, location)
            % SETLOITERLOCATION function updating the required loiter
            % location for this task.
            obj.LoiterLocation.updatePositionObject(location);
        end
        
        function [] = graphTaskProperties(obj, axes, cmap)
            textOffset = 0.25; %distance offset used when labeling the agents/tasks

            startTask = obj.LoiterLocation;
            plot3(axes, startTask.x, startTask.y, startTask.z,'+','color',cmap(ADCA_TaskTypes.TASK_LOITER,:));
            text(startTask.x + textOffset, startTask.y + textOffset, startTask.z + textOffset, ['T' num2str(obj.m_TaskKey.TASK_ID)], 'Parent', axes);
        end
    end
    
end