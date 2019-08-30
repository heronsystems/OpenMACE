clc
clear all
close all

for force = 0:1:10
    rho = -1 * (force/10 * (1-0.2)-1);
    for i = 1:1:361
        response(i) = (rho*10*0.7*0.5) / (1-0.7*cos((i-1) * 3.14 / 180.0));
        x_response(i) = response(i) * cos((i-1) * 3.14 / 180.0);
        y_response(i) = response(i) * sin((i-1) * 3.14 / 180.0);
    end
    plot(x_response,y_response)
    hold on
end

for force = 0:1:10
    rho = -1 * (force/10 * (1-0.2)-1);
    for i = 1:1:361
        response(i) = (rho*5*0.5*0.5) / (1-0.5*cos((i-1) * 3.14 / 180.0));
        x_response(i) = response(i) * cos((i-1) * 3.14 / 180.0);
        y_response(i) = response(i) * sin((i-1) * 3.14 / 180.0) - 5;
    end
    plot(x_response,y_response)
    hold on
end

for force = 0:1:10
    rho = -1 * (force/10 * (1-0.2)-1);
    for i = 1:1:361
        response(i) = (rho*5*0.3*0.5) / (1-0.3*cos((i-1) * 3.14 / 180.0));
        x_response(i) = response(i) * cos((i-1) * 3.14 / 180.0);
        y_response(i) = response(i) * sin((i-1) * 3.14 / 180.0) - 10;
    end
    plot(x_response,y_response)
    hold on
end

grid on

