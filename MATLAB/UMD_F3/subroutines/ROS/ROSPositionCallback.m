function ROSPositionCallback(src,msg,ROS_MACE)

    global agentPosition;
%     tic;
    
    agentIndex = ROS_MACE.agentIDtoIndex( msg.VehicleID );
    [xF3, yF3] = ENUtoF3( msg.Easting , msg.Northing );
    agentPosition(agentIndex,:) = [xF3, yF3, msg.Altitude];
    
%     positionCallback( ROS_MACE, msg);
%     fprintf('Time is %f\n',toc);

    

end