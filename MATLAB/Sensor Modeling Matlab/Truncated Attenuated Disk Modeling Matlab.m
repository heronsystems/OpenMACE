clear all
clc

alpha = 0.5;
beta = 1;
maxSensingRange = 10;
uncertainRange = 6;
certainRange = maxSensingRange - uncertainRange;
figure('Name','Truncated Attenuated Disk Response: Alpha Influence');
hold all;
for alpha = 0.0:0.5:4
    distanceX = [];
    response = [];
    for distance = 0.01:0.01:11
        [row,column] = size(response);
        index = column + 1;
        distanceX(index) = distance;
        if(distance <= certainRange)
            response(index) = 1;
        elseif((distance > certainRange) && (distance<=maxSensingRange))
            response(index) = exp(-alpha * (distance - certainRange)^beta);
        else
            response(index) = 0;
        end
    end
    legendTitle = strcat('alpha = ',num2str(alpha));
    plot(distanceX,response,'color',rand(1,3),'DisplayName',legendTitle)
    hold all
    pause(2)
end

alpha = 1;
figure('Name','Truncated Attenuated Disk Response: Beta Influence');
hold all;
for beta = 0.0:0.5:4
    distanceX = [];
    response = [];
    for distance = 0.01:0.01:11
        [row,column] = size(response);
        index = column + 1;
        distanceX(index) = distance;
        if(distance <= certainRange)
            response(index) = 1;
        elseif((distance > certainRange) && (distance<=maxSensingRange))
            response(index) = exp(-alpha * (distance - certainRange)^beta);
        else
            response(index) = 0;
        end
    end
    legendTitle = strcat('alpha = ',num2str(alpha));
    plot(distanceX,response,'color',rand(1,3),'DisplayName',legendTitle)
    hold all
    pause(2)
end