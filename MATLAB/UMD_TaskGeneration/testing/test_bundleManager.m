% use it after running the simulation/experiment to see how the error decreases

for i = 1:ROS_MACE.N
%     error{i} = zeros(1,size(agentStateHist{i},2));
%     for k = 1:size(error{i},2)
%         index = max(find(agentStateHist{i}(1,k)-agentTargetHist{i}(1,:)>=0));
%         error{i}(k) = norm(agentStateHist{i}(2:3,k)-agentTargetHist{i}(2:3,index));
%     end
%     figure;
%     plot(agentStateHist{i}(1,:),error{i});
%     hold on;
%     stem(agentTargetHist{i}(1,:),ones(size(agentTargetHist{i}(1,:))));
%     grid on;
%     title(['Quad ' num2str(i) ' horizontal error (capture radius = 1m)']);
%     xlabel('Time (s)');
%     ylabel('Distance (m)');
%     legend('error','new position command');
    figure;
    plot(agentStateHist{i}(2,:),agentStateHist{i}(3,:),'b:'); hold on;
    plot(agentTargetHist{i}(2,:),agentTargetHist{i}(3,:),'b-');
    grid on;    
    axis equal;
    xlabel('F3 x (m)');
    ylabel('F3 y (m)');
    legend('Trajectory','Bundle path');
    title(['Quad' num2str(i)]);
end