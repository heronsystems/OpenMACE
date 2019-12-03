function ROSAngleCallback(src,msg,agentIDtoIndex)

    global agentYawAngle;
%     tic;
    
    agentIndex = agentIDtoIndex( msg.VehicleID );
    agentYawAngle(agentIndex) = -msg.Yaw + 2.3736;
    
%     fprintf('Time is %f\n',toc);

end