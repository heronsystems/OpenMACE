classdef RoboticAgent_Params < handle
    properties
        AGENT_ID = 0;
        AGENT_TYPE = ADCA_AgentTypes.UAV_PLANE;
        CompatibilityMatrix = [];

        FuelCost = 1;
        NominalVelocity = 1;
    end
    
    methods
        function RoboticAgent_Params = RoboticAgent_Params(id, type)
            RoboticAgent_Params.AGENT_ID = id;
            RoboticAgent_Params.AGENT_TYPE = type;
            RoboticAgent_Params.FuelCost = 1;
            
            RoboticAgent_Params.CompatibilityMatrix = ADCA_TaskTypes.initialize_CompatibilityMatrix(type);

            %RoboticAgent_Params.FuelCost = 1 * rand;
        end
    end
end