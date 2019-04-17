function wptManager( ROS_MACE, wpts, captureRadius)

numWpts = size(wpts{1},1);
for curWpt = [1:1:numWpts]
    % plot
    subplot(ROS_MACE.taskAndLocation);
     wptsDesired = [];
    for i = 1:1:ROS_MACE.N
        wptsDesired = [ wptsDesired; wpts{i}(curWpt,:) ];
    end
    plot(wptsDesired(:,1), wptsDesired(:,2),'k+','MarkerSize',4,'linewidth',2);
    drawnow;
    
    % send
    updateWpts( ROS_MACE, wptsDesired )
    % wait
    waitForWptsToBeReached( ROS_MACE, wptsDesired, captureRadius )
    disp('**** WPTS ACHIEVED ****');
    disp('Keeping Station for 5 seconds...');
    countdownVerbose(5);
end


end