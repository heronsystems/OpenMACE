classdef location < matlab.mixin.Copyable
    properties
        FRAME = [];
        x = 0;
        y = 0;
        z = 0;
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            % Make a deep copy of all of the objects
            cpObj = location();
            cpObj.updatePositionExplicit(obj.x,obj.y,obj.z);
%             cpObj.x = obj.x;
%             cpObj.y = obj.y;
%             cpObj.z = obj.z;
        end
    end
    
    methods
        
        function [] = printLocation(obj)
            displayValue = ['Location: ',num2str(obj.x),' : ', num2str(obj.y)];
            disp(displayValue);
        end
        function location = location()
            location.x = 0;
            location.y = 0;
            location.z = 0;
        end
        
        function updatePositionObject(obj,newPosition)
            obj.x = newPosition.x;
            obj.y = newPosition.y;
            obj.z = newPosition.z;
        end
        
        function updatePositionExplicit(obj,xLoc,yLoc,zLoc)
            obj.x = xLoc;
            obj.y = yLoc;
            obj.z = zLoc;
        end
        
        function positionVector = getPositionVector(obj)
            positionVector = [obj.x obj.y obj.z];
            positionVector = positionVector';
        end
        
        function result = minus(obj1,obj2)
            result = zeros(3,1);
            result(1,1) = obj1.x - obj2.x;
            result(2,1) = obj1.y - obj2.y;
            result(3,1) = obj1.z - obj2.z;
        end
    end
    
    methods
        function distance = computeDistance_2D(obj, location)
            difference = obj - location;
            distance = sqrt(difference(1,1)^2 + difference(2,1)^2);
        end
        
        function distance = computeDistance_3D(obj, location)
            difference = obj - location;
            distance = sqrt(sum(difference.^2));
        end
    end
    
end
