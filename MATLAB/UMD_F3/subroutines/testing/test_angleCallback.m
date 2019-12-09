function test_angleCallback(src,msg,agentIDtoIndex)
    persistent n;
    global agentAngle;
    tic;
    if isempty(n)
        n = 0;
    end
    
    agentIndex = agentIDtoIndex( msg.VehicleID );
    agentAngle(agentIndex) = msg.Yaw;
    n = n+1;
    fprintf('msg No.%d received.\n', n);
    fprintf('Time is %f\n',toc);
    agentAngle
end