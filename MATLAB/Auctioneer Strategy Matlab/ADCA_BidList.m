classdef ADCA_BidList < ADCA_AbstractGenericList
    
    methods 
        function [obj] = createEmptyList(obj)
            if isempty(obj.List)
                obj.List = ADCA_BidDescriptor();
            end
        end
        
        function [bidObject] = getBidAtIndex(obj,index)
            bidObject = obj.getItemAtIndex(index);
        end
        
        function [] = appendBidToList(obj, newBid)
            obj.appendToList(newBid);
        end
        
        function listSize = sizeOfBidList(obj)
            listSize = obj.sizeOfList();
        end
        
        function [task] = removeBidAtIndex(obj, index)
            task = obj.removeAtIndex(index);
        end
       
        function [bidList] = getCurrentBidList(obj)
            bidList = obj.getCurrentList;
        end
    end
    
end