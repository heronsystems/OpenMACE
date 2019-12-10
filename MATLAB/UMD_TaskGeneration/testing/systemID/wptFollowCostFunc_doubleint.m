function [cost] = wptFollowCostFunc_doubleint(p,t,xhist,xd,yd,B,N)

% update free parameters
kp = p(1);
kd = p(2);
umax = p(3);
vmax = p(4);
%params.delay = p(5);

% 
d = umax/vmax; % agent damping parameter
A = [0 0 1 0; 0 0 0 1; 0 0 -d 0; 0 0 0 -d];
x0 = xhist(1,:);
xsim = simulateSys_doubleint(x0,t,xd,yd,kp,kd,umax,A,B,N);


% compute residual error
for j = 1:1:N
   diffX = xsim(:,4*j-3) - xhist(:,4*j-3);
   diffY = xsim(:,4*j-2) - xhist(:,4*j-2);
   res(j) = sum(sqrt((diffX).^2 + (diffY).^2))/length(diffX);
end

cost = sum(res)/N;





end