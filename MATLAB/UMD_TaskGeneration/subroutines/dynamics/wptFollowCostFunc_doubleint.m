function [cost] = wptFollowCostFunc_doubleint(p,params,thist,xhist,xd,yd)

% update free parameters
params.kp = p(1);
params.kd = p(2);
params.umax = p(3);
params.vmax = p(4);
%params.delay = p(5);

% 
params.d = params.umax/params.vmax; % agent damping parameter
params.A = [0 0 1 0; 0 0 0 1; 0 0 -params.d 0; 0 0 0 -params.d];



% compute residual error
for j = 1:1:N
   diffX = xsim(:,4*j-3) - xhist(:,4*j-3);
   diffY = xsim(:,4*j-2) - xhist(:,4*j-2);
   res(j) = norm(abs(diffX) + abs(diffY));
end

cost = sum(res);





end