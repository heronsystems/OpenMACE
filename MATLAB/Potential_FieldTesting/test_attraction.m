clc
clear all
close all

target = [10,10];

figure()
gain_attraction = 1; gain_zero = 0.01; gain_transition = 0.2;
for gain_attraction = 1:1:10
    result = [];
for i = 0:0.1:10
        pos = [i,i];
        distance = norm(target - pos);
        f_att = gain_attraction / (1+(exp(-gain_transition * distance)/gain_zero));
        force = f_att * (target - pos) / distance;
        result = [result;force];
    end
    normalized_result = vecnorm(result,2,2);
    plot(normalized_result)
    hold on
end

figure()
gain_attraction = 2; gain_zero = 0.01; gain_transition = 0.2;
for gain_zero = 0.01:0.01:0.1
    result = [];
for i = 0:0.1:10
        pos = [i,i];
        distance = norm(target - pos);
        f_att = gain_attraction / (1+(exp(-gain_transition * distance)/gain_zero));
        force = f_att * (target - pos) / distance;
        result = [result;force];
    end
    normalized_result = vecnorm(result,2,2);
    plot(normalized_result)
    hold on
end

figure()
gain_attraction = 2; gain_zero = 0.01; gain_transition = 0.5;
for gain_transition = 0.2:0.1:1
    result = [];
for i = 0:0.1:10
        pos = [i,i];
        distance = norm(target - pos);
        f_att = gain_attraction / (1+(exp(-gain_transition * distance)/gain_zero));
        force = f_att * (target - pos) / distance;
        result = [result;force];
    end
    normalized_result = vecnorm(result,2,2);
    plot(normalized_result)
    hold on
end

figure()
gain_attraction = 200; gain_zero = 0.001; gain_transition = 0.5;
result = [];
for i = 6:0.1:10
    pos = [i,i];
    distance = norm(target - pos);
    f_att = gain_attraction / (1+(exp(-gain_transition * distance)/gain_zero));
    force = f_att * (target - pos) / distance;
    result = [result;force];
end
normalized_result = vecnorm(result,2,2);
plot(normalized_result)
hold on
