function [xdot] = swarmDynamics(t,x,params)
global bhvState;
for i = 1:1:params.N
    xi = [ x(3*i-2); x(3*i-1); x(3*i)];
    xdot(3*i-2,1) = params.v*cos(xi(3));
    xdot(3*i-1,1) = params.v*sin(xi(3));
    if ( strcmp('goStraight',params.bhv.name ) )
        turnRateCmd = 0;
    elseif ( strcmp('followNetwork', params.bhv.name ) && strcmp(bhvState{i}.status,'nominal') )
        turnRateCmd = 0;        
    elseif ( strcmp('followNetwork', params.bhv.name ) && strcmp(bhvState{i}.status,'following') )
        nodesInView = nodesInFOV(xi(1),xi(2),params.nodeX,params.nodeY,params.Rsense);
        turnRateCmd = followNetworkTurnRateCmd(xi(1),xi(2),xi(3),params.nodeX(nodesInView),params.nodeY(nodesInView),params.hDiffTol, params.uMax);
    elseif ( strcmp('followNetwork', params.bhv.name ) && strcmp(bhvState{i}.status,'refracting') )
        turnRateCmd = bangBangHeadingController( xi(3), bhvState{i}.randomHeading, params.hDiffTol, params.uMax );
    end
    xdot(3*i,1) = turnRateCmd;
end

end