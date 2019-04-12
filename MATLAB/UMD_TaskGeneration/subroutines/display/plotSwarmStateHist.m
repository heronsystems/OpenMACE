function plotSwarmStateHist(swarmStateHist, swarmModel)

M = length(swarmStateHist);

% initialize
for i = 1:1:swarmModel.N
    agent{i}.x = [];
    agent{i}.y = [];
    agent{i}.xd = [];
    agent{i}.yd = [];
end


for k = 1:1:M
    swarmState = swarmStateHist{k};
    for i = 1:1:swarmModel.N
        xk = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i) ];
        agent{i}.x = [agent{i}.x xk(1)];
        agent{i}.y = [agent{i}.y xk(2)];
        agent{i}.xd = [agent{i}.xd swarmState.xd(i)];
        agent{i}.yd = [agent{i}.yd swarmState.yd(i)];
    end
end

figure;
colorString = 'rgbk';
for i = 1:1:swarmModel.N
    subplot(2,1,1)
    plot(agent{i}.x,'linewidth',3,'Color',colorString(i)); hold on;
    plot(agent{i}.xd,'o','linewidth',1,'Color',colorString(i))
    grid on;
    
    subplot(2,1,2)
    plot(agent{i}.y,'linewidth',3,'Color',colorString(i)); hold on;
    plot(agent{i}.yd,'o','linewidth',1,'Color',colorString(i))
    grid on;
    
end

subplot(2,1,1)
ylabel('X')
xlabel('Time-Step')
set(gca,'FontSize',16)
axis tight;

subplot(2,1,2)
ylabel('Y')
xlabel('Time-Step')
set(gca,'FontSize',16)
axis tight;


