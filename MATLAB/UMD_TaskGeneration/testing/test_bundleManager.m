% use it after running the simulation/experiment to see how the error decreases

for i = 1
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
    plot(agentStateHist{i}(2,:),agentStateHist{i}(3,:),'r-'); hold on;
    plot(agentTargetHist{i}(2,:),agentTargetHist{i}(3,:),'b-');
    plot(agentTargetHist{i}(2,1),agentTargetHist{i}(3,1),'bo','MarkerSize',12);
    plot(agentTargetHist{i}(2,end),agentTargetHist{i}(3,end),'bs','MarkerSize',12);
    grid on;    
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    legend('Trajectory','Bundle path','Start','End');
    title(['Quad' num2str(i)]);
    set(gca,'FontSize',16);
    
    figure;
%     plot(agentStateHist{i}(2,:),agentStateHist{i}(3,:),'r-'); hold on;
    plot(agentTargetHist{i}(2,:),agentTargetHist{i}(3,:),'b-');hold on;
    plot(agentTargetHist{i}(2,1),agentTargetHist{i}(3,1),'bo','MarkerSize',12);
    plot(agentTargetHist{i}(2,end),agentTargetHist{i}(3,end),'bs','MarkerSize',12);
    grid on;    
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    legend('Bundle path','Start','End');
    title(['Quad' num2str(i)]);
    set(gca,'FontSize',16);
    

    t = agentStateHist{i}(1,:);
    x = agentStateHist{i}(2,:);
    y = agentStateHist{i}(3,:);
    xdraw = agentTargetHist{i}(2,:);
    ydraw = agentTargetHist{i}(3,:);
    traw = agentTargetHist{i}(1,:);
    xd = interp1(traw,xdraw,t,'previous');
    yd = interp1(traw,ydraw,t,'previous');
    vx = diff(x)./diff(t);
    vy = diff(y)./diff(t);
    v = sqrt(vx.^2 + vy.^2);
    r = sqrt( (x-xd).^2 + (y-yd).^2 );
    
    figure(10);
    subplot(2,1,1);
    plot(t(1:end-1),v);
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    grid on;
    set(gca,'FontSize',16)
    hold on;
    plot(t(1:end-1),diff(t))%lowpass(v,0.1));
    subplot(2,1,2);
    plot(t,r);
    hold on;
    plot(traw, ones(size(xdraw)),'mo');
    xlabel('Time (s)');
    ylabel('Range (m)');
    grid on;
    set(gca,'FontSize',16)
    
    figure(11);
    subplot(2,1,1);
    plot(traw,xdraw,'ro');
    hold on;
    plot(t,xd,'b')
    plot(t,x,'k');
    xlabel('Time (s)');
    ylabel('X (m)');
    grid on;
    legend('new waypoint command','target waypoint','trajectory')
    set(gca,'FontSize',16);
    
    subplot(2,1,2);
    plot(traw,ydraw,'ro');
    hold on;
    plot(t,yd,'b')
    plot(t,y,'k');
    xlabel('Time (s)');
    ylabel('Y (m)');
    grid on;
    legend('new waypoint command','target waypoint','trajectory')
    set(gca,'FontSize',16)
    
    
end