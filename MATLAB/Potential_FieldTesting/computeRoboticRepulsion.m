function [distance] = computeRoboticRepulsion(force, theta)

agent_radius = 0.05; %R_r
agent_speed = 0.07; %v_r
agent_max_speed = 0.08; %v_max
environmental_influence = 1.25; %denoted as C which is a positive number > 1 capturing the environmental influence on the force
repulsive_gain = 10; %denoted as P which is a positive constant determining the magnitude of the repuslive force
obstacle_proximity = 0.2; %defines how close the robot can be to other robots / obstacles as a value of 0 < p_0 < 1
velocity_proportion = agent_speed / (agent_max_speed * environmental_influence); %denoted as E_r
distance_gain = 5; %denoted as k

z = -force/repulsive_gain*(1-obstacle_proximity) + 1;

numerator = z * distance_gain * agent_radius * velocity_proportion;
denominator = 1 - velocity_proportion * cos(theta);

distance = numerator / denominator;
end


