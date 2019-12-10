%test computing stopping distance

max_acceleration = -5;
response_time = 1.1;

response  = [];
for i = 0:0.01:10
response_velocity = -max_acceleration * response_time + ...
    sqrt(2*max_acceleration*i + max_acceleration^2*response_time);

    response = [response;response_velocity];
    
end

plot(response)
