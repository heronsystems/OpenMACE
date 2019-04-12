classdef ADCA_BiddingBundle < handle
    
    properties
        m_CurrentBundle = []; %the current bidding task bundle
        minBidEpsilon = 0.1; %the minimum value required to cause an agent to the task worth bidding on and trying to achieve in the network.
    end
    
    methods
        
        function ADCA_BiddingBundle = ADCA_BiddingBundle()
            ADCA_BiddingBundle.m_CurrentBundle = ADCA_BidList();
        end
        
        function [bundleWorkload] = getBiddingBundleWorkload(obj)
            currentList = obj.m_CurrentBundle.getCurrentBidList();
            bundleWorkload = 0;
        end
        
        function [bundleSize] = getBiddingBundleSize(obj)
            bundleSize = obj.m_CurrentBundle.sizeOfBidList();
        end
        
        function [bid] = getBidAtIndex(obj, index)
            bid = obj.m_CurrentBundle.getBidAtIndex(index);
        end
        
        function [index] = findTaskIndex(obj, taskKey)
            index = [];
            for i = 1:1:obj.m_CurrentBundle.sizeOfBidList()
                bid = obj.getBidAtIndex(i);
                if(bid.m_TaskKey == taskKey)
                    index = i;
                    break;
                end
            end
        end
        
        function [] = invalidateSubsequentBids(obj, index)
            for i = index:1:obj.m_CurrentBundle.sizeOfBidList()
                bid = obj.getBidAtIndex(i);
                bid.updateValidity(false);
            end
        end
        
        function [sizeExceeded] = isBundleSizeExceeded(obj, limit)
            if(obj.getBiddingBundleSize() >= limit)
                sizeExceeded = true;
            else
                sizeExceeded = false;
            end
        end
        
        function [newList] = appendNewBid(obj,bid)
            obj.m_CurrentBundle.appendBidToList(bid);
            newList = obj.m_CurrentBundle;
        end
        
        function [doesExist, keyIndex] = doesBundleContainKey(obj, taskKey)
            doesExist = false;
            if(obj.getBiddingBundleSize() > 0)
                for index = 1:1:obj.getBiddingBundleSize()
                    bid = obj.getBidAtIndex(index);
                    if(bid.m_TaskKey == taskKey)
                        doesExist = true;
                        keyIndex = index;
                        break;
                    end
                end
            end
        end
        
        function [taskKeys] = getTaskKeys(obj)
            if(obj.getBiddingBundleSize() <= 0)
                taskKeys = [];
                return;
            end
            
            for i = 1:1:obj.getBiddingBundleSize()
                bid = obj.getBidAtIndex(i);
                taskKeys(1,i) = bid.m_TaskKey;                
            end
        end
    end
    
end
