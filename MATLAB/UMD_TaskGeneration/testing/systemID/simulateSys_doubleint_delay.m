function xsim = simulateSys_doubleint_delay(x0,t,xd,yd,kp,kd,umax,A,B,N,delay)

params.kp = kp;
params.kd = kd;
params.umax = umax;
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
    if ( t(k) > delay )
        tdelay = t(k)- delay;
        params.xd  = interp1(t,xd,tdelay,'previous');
        params.yd = interp1(t,yd,tdelay,'previous');
    else               
        params.xd = xd(k,:);
        params.yd = yd(k,:);
    end    
    [~,xout] = ode45(@(t,x) swarmWptODE(t,x,params),[0 delt],x0);
    xsim(k,:)= xout(end,:);
    x0 = xsim(k,:);
end

end