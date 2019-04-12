classdef (Abstract) ADCA_AbstractGenericList < handle
    
    properties
        List = [];
    end
    
    methods (Abstract)
        [obj] = createEmptyList(obj)
    end
    
    methods (Access = protected)
        function ADCA_AbstractGenericList = ADCA_AbstractGenericList()
            ADCA_AbstractGenericList = ADCA_AbstractGenericList.createEmptyList();
        end    
        
        function [item] = getItemAtIndex(obj,index)
            if((index) > obj.sizeOfList())
                item = [];
            else
                item = obj.List(index+1,1);
            end
        end
        
        function appendToList(obj, newItem)
            index = obj.sizeOfList()+2;
            obj.List(index,1) = newItem;
        end
        
        function listSize = sizeOfList(obj)
            [rows,columns] = size(obj.List);
            listSize = rows - 1;
        end
        
        function [task] = removeAtIndex(obj, index)
            task = obj.List(index + 1,:);
            obj.List(index + 1,:) = [];
        end
        
        function [list] = getCurrentList(obj)
            list = obj.List;
            list(1,:) = [];
        end
    end
    
end