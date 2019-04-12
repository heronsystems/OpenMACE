classdef ADCA_TaskSurveyDescriptor < ADCA_TaskDescriptor
    
    properties
        OriginLocation = location(0,0,0);
    end
    
    methods
        function setOriginLocation(obj, location)
            obj.OriginLocation.updatePositionObject(location);
        end
    end
    
end
