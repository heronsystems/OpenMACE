classdef ADCA_TaskTypes < uint32
    enumeration
        TASK_SURVEY (1)
        TASK_LOITER (2)
        UNKNOWN (3)
    end
    methods(Static)
        function Compatibility_Matrix = initialize_CompatibilityMatrix(agentType)
            % INITIALIZE_COMPATIBILITYMATRIX Determines the compatibiility
            % of a given agent type to address the available task. The
            % return will be a single row matrix consisting of the tasks
            % the agent type could satisfy during the auction process.
            % [AddressTask_1, AddressTask_2, ...] [TaskType_1,
            % TaskType_2, ...]^T
            Compatibility_Matrix = zeros(1,uint32(ADCA_TaskTypes.UNKNOWN) - 1);
            
            switch agentType
                case ADCA_AgentTypes.UAV_PLANE
                    Compatibility_Matrix(1,uint32(ADCA_TaskTypes.TASK_SURVEY))  = 1;
                    Compatibility_Matrix(1,uint32(ADCA_TaskTypes.TASK_LOITER))  = 0;
                case ADCA_AgentTypes.UAV_ROTARY
                    Compatibility_Matrix(1,uint32(ADCA_TaskTypes.TASK_SURVEY))  = 0;
                    Compatibility_Matrix(1,uint32(ADCA_TaskTypes.TASK_LOITER))  = 1;
                    
                otherwise
                    warning('Nothing implemented yet for specified agent type.')
            end
            
        end
    end
end