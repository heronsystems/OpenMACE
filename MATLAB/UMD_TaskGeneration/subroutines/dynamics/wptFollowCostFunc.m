function [cost] = wptFollowCostFunc(p,params,thist,xhist,xd,yd)

% update free parameters
params.kp = p(1);
params.kd = p(2);
params.umax = p(3);
params.vmax = p(4);
params.delay = p(5);

% 
params.d = params.umax/params.vmax; % agent damping parameter
params.A = [0 0 1 0; 0 0 0 1; 0 0 -params.d 0; 0 0 0 -params.d];

% simulate
N = params.N;
numPts = length(thist);
xsim(1,:) = xhist(1,:);
for k = 2:1:numPts
    delt  = thist(k) - thist(k-1);
    %x0 = [ xsim(k-1,4*j-3); xsim(k-1,4*j-2); xsim(k-1,4*j-1); xsim(k-1,4*j)];
    x0 = xsim(k-1,:);
    % lookup waypoint desired
    if ( thist(k) > params.delay )
        tdelay = thist(k)-params.delay;
        params.xd  = interp1(thist,xd,tdelay);
        params.yd = interp1(thist,yd,tdelay);
        
    else               
        params.xd = xd(k,:);
        params.yd = yd(k,:);
    end
    % call ode45
    [tout,xout] = ode45(@(t,x) swarmWptODE(t,x,params),[0 delt],x0);
    % package output
    xsim(k,:)= xout(end,:);
end

% compute residual error
for j = 1:1:N
   diffX = xsim(:,4*j-3) - xhist(:,4*j-3);
   diffY = xsim(:,4*j-2) - xhist(:,4*j-2);
   res(j) = norm(abs(diffX) + abs(diffY));
end

cost = sum(res);





end