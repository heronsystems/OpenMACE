% Update the real-time task bundle during flight test at F3
% Sheng Cheng, 2019 April
function ROS_MACE = plotTaskBundleRealTime(swarmWorld, swarmState, ROS_MACE)

subplot(ROS_MACE.taskAndLocation);
colors=['rbkmgcy'];


if isempty(ROS_MACE.tempHandle{1,1})
    for k = 1:ROS_MACE.N
        ROS_MACE.tempHandle{1,k} = plot(swarmWorld.cellCenterOfMass(swarmState.wptList(k,:),1),swarmWorld.cellCenterOfMass(swarmState.wptList(k,:),2),[colors(k) '-']);
        ROS_MACE.tempHandle{2,k} = plot(swarmWorld.cellCenterOfMass(swarmState.wptList(k,swarmState.wptIndex(k)),1),swarmWorld.cellCenterOfMass(swarmState.wptList(k,swarmState.wptIndex(k)),2),[colors(k) '+'],'MarkerSize',4,'linewidth',2);
    end
else
    for k = 1:ROS_MACE.N
        set(ROS_MACE.tempHandle{1,k},'XData', swarmWorld.cellCenterOfMass(swarmState.wptList(k,:),1),'YData',swarmWorld.cellCenterOfMass(swarmState.wptList(k,:),2));
        set(ROS_MACE.tempHandle{2,k},'Xdata', swarmWorld.cellCenterOfMass(swarmState.wptList(k,swarmState.wptIndex(k)),1),'YData',swarmWorld.cellCenterOfMass(swarmState.wptList(k,swarmState.wptIndex(k)),2));
    end
end
drawnow;