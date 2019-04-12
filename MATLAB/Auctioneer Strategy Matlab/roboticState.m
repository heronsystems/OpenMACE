classdef roboticState < matlab.mixin.Copyable
    properties
        POSITION = [];
        TIME = 0;
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            cpObj = roboticState();
            % Make a deep copy of all of the objects
            cpObj.POSITION = copy(obj.POSITION);
            cpObj.TIME = obj.TIME;
        end
    end
    
    methods
        function roboticState = roboticState()
            roboticState.POSITION = location();
            roboticState.TIME = 0;
        end
        
        function [] = print(obj)
           fprintf('Position X:%f Y:%f Z:%f \n',obj.POSITION.x, obj.POSITION.y, obj.POSITION.z);
           fprintf('Time:%f \n',obj.TIME);
        end
    end
    
end