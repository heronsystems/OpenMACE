%Every agent should call this function to initialize the Auction Params
classdef ADCA_AuctionParams
    properties
        MAX_BUNDLESIZE = 2;
%         MAX_ASSIGNMENTSIZE = 3;
        MAX_ASSIGNMENTSIZE = 6; % Sheng: make it larger
        MAX_WORKLOAD = 100;
    end
    methods
        function ADCA_AuctionParams = ADCA_AuctionParams()
        end
        
    end
end


