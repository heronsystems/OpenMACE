function xsim = simulateSys_tripleint(x0,t,xd,yd,kp,kd,jmax,A,B,N)

params.kp = kp;
params.kd = kd;
params.umax = jmax;
params.xd = xd;
params.yd = yd;
params.N = N;
params.A = A;
params.B = B;

% simulate
numPts = length(t);
xsim(1,:) = x0;
for k = 2:1:numPts
    delt  = t(k) - t(k-1);           
    params.xd = xd(k,:);
    params.yd = yd(k,:);
    [~,xout] = ode45(@(t,x) swarmWptODE_tripleInt(t,x,params),[0 delt],x0);
    xsim(k,:)= xout(end,:);
    x0 = xsim(k,:);
end

end