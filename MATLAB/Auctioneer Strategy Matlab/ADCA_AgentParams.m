
classdef ADCA_AgentParams
    properties
        AGENT_ID = 0;
        AGENT_TYPE = ADCA_AgentTypes.UAV_PLANE;
        CompatibilityMatrix = [];
    end
    methods
        %Every agent should call this function to initialize the Auction Params
        function ADCA_AgentParams = ADCA_AgentParams(ID, type)
            %Initialize the type of agent
            if(isenum(type))
                ADCA_AgentParams.AGENT_TYPE = type;
                ADCA_AgentParams.CompatibilityMatrix = ADCA_TaskTypes.initialize_CompatibilityMatrix(type);
            end
            ADCA_AgentParams.AGENT_ID = ID;
            
        end
    end
    
end

