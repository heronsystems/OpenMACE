function xdot = swarmWptODE_tripleInt(t,x,params)
kp = params.kp;
kd = params.kd;
umax = params.umax;
xd = params.xd;
yd = params.yd;
N = params.N;
A = params.A;
B = params.B;

for j = 1:1:N
    xj = [ x(6*j-5); x(6*j-4); x(6*j-3); x(6*j-2); x(6*j-1); x(6*j)];
    u = waypointController(xj,xd(j),yd(j),kp,kd,umax);
    xdot(6*(j-1)+1:6*j,1) = A*xj + B*u;
end
end