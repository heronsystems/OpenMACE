function xdot = swarmDynamics(swarmState,swarmModel)


N = swarmModel.N;
for i = 1:1:N
    switch swarmModel.communicationTopology
        case 'allToAll'
            % package state vector of i-th agent
            xi = [ swarmState{i}.x(1); swarmState{i}.x(2); swarmState{i}.x(3); swarmState{i}.x(4)];
            % waypoint controller
            u = waypointController(xi,swarmState{i}.xd,swarmState{i}.yd,swarmModel.kp_wpt,swarmModel.kd_wpt,swarmModel.umax);    
            % double-integrator dynamics with damping and saturation
            xdot{i} = swarmModel.A*xi + swarmModel.B*u;
        case 'centralized'
            % package state vector of i-th agent
            xi = [ swarmState.x(4*i-3); swarmState.x(4*i-2); swarmState.x(4*i-1); swarmState.x(4*i)];
            % waypoint controller
            u = waypointController(xi,swarmState.xd(i),swarmState.yd(i),swarmModel.kp_wpt,swarmModel.kd_wpt,swarmModel.umax);    
            % double-integrator dynamics with damping and saturation
            xdot(4*(i-1)+1:4*i,1) = swarmModel.A*xi + swarmModel.B*u;
    end
    
end
end
