function xdot = swarmWptODE(t,x,params)
kp = params.kp;
kd = params.kd;
umax = params.umax;
xd = params.xd;
yd = params.yd;
N = params.N;
A = params.A;
B = params.B;

for j = 1:1:N
    xj = [ x(4*j-3); x(4*j-2); x(4*j-1); x(4*j)];
    u = waypointController(xj,xd(j),yd(j),kp,kd,umax);
    xdot(4*(j-1)+1:4*j,1) = A*xj + B*u;
end

% saturate speed?

end