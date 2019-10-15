%% Test computing stopping distance
clc
clear all
close all

max_acceleration = 5;
response_time = 1.1;

response  = [];
x_vec  = [];
for i = 0:0.01:10
    x_vec = [x_vec;i];
    response_velocity = -max_acceleration * response_time + ...
        sqrt(2*max_acceleration*i + max_acceleration^2*response_time);
    
    response = [response;response_velocity];
    
end
figure()
plot(x_vec, response)


%% Attraction for steering vector
attraction_gain = 1.8;
gain_1 = 0.6;
gain_2 = 1.25;

response  = [];
x_vec  = [];

for i = 0 : 0.1 : 10
    x_vec = [x_vec;i];
    responseAttraction = attraction_gain * pi/4 * (exp(-gain_1 * i) + gain_2);
    response = [response;responseAttraction];
end

figure() 
plot(x_vec, response)