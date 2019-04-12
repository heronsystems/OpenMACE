classdef RobotEnvironment < handle
    
    properties
        X_MAX = 10;
        X_MIN = -10;
        Y_MAX = 10;
        Y_MIN = -10;
        Z_MAX = 10;
        Z_MIN = 0;
        
        operationsFigure = [];
        colorMapVehicles = [];
        colorMapTasks = [];
    end
    
    methods
        function RobotEnvironment = RobotEnvironment(x_min, x_max, y_min, y_max, z_min, z_max)
            RobotEnvironment.X_MAX = x_max;
            RobotEnvironment.Y_MAX = y_max;
            RobotEnvironment.Z_MAX = z_max;
            
            RobotEnvironment.X_MIN = x_min;
            RobotEnvironment.Y_MIN = y_min;
            RobotEnvironment.Z_MIN = z_min;
                      
            RobotEnvironment.operationsFigure = figure('Name', 'Initial Agent / Task Environment');
            xlabel('X Position (m)') % x-axis label
            ylabel('Y Position (m)') % y-axis label
            zlabel('Z Position (m)') % z-axis label
            
            grid on;
            hold on;
            
            %update the graph axis with the operational dimensions
            axis([x_min, x_max, y_min, y_max, z_min, z_max])
            
            RobotEnvironment.colorMapVehicles = colormap('summer'); %colormap available for differentiating vehicles
            RobotEnvironment.colorMapTasks = colormap('autumn'); %colormap available for differentiating tasks
            
        end
        
        function [position] = getRandomLocation(obj)
            xPos = rand(1) * (obj.X_MAX - obj.X_MIN) + obj.X_MIN;
            yPos = rand(1) * (obj.Y_MAX - obj.Y_MIN) + obj.Y_MIN;
            zPos = rand(1) * (obj.Z_MAX - obj.Z_MIN) + obj.Z_MIN;
            
            position = location();
            position.updatePositionExplicit(xPos,yPos,zPos);
        end
        
        % Sheng: add for testing evenly spaced agents and tasks
        function [position] = getEquidistantLocation(obj,id,totalNum,ratio)
            xPos = id / totalNum * (obj.X_MAX - obj.X_MIN) + obj.X_MIN;
            yPos = ratio * (obj.Y_MAX - obj.Y_MIN) + obj.Y_MIN;
            zPos = ratio * (obj.Z_MAX - obj.Z_MIN) + obj.Z_MIN;
            
            position = location();
            position.updatePositionExplicit(xPos,yPos,zPos);
        end
        
        % Sheng: add for loading location of agents
        function [position] = loadAgentLocation(obj,agentLocation)
            position = location();
            position.updatePositionExplicit(agentLocation(1),agentLocation(2),agentLocation(3));
        end
        
        % Sheng: add for loading location of tasks
        function [position] = loadTaskLocation(obj,taskLocation)
            position = location();
            position.updatePositionExplicit(taskLocation(1),taskLocation(2),taskLocation(3));
        end
    end
end