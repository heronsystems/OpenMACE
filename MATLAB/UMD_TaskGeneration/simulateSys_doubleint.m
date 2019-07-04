function xsim = simulateSys_doubleint(x0,thist,xd,yd)

% simulate
numPts = length(thist);
xsim(1,:) = x0;
for k = 2:1:numPts
    delt  = thist(k) - thist(k-1);           
    params.xd = xd(k,:);
    params.yd = yd(k,:);
    [~,xout] = ode45(@(t,x) swarmWptODE(t,x,params),[0 delt],x0);
    xsim(k,:)= xout(end,:);
end

end