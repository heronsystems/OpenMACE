function [cost] = wptFollowCostFunc_tripleint(p,t,xhist,xd,yd,B,N)

% update free parameters
kp = p(1);
kd = p(2);
vmax = p(3);
amax = p(4);
jmax = p(5);

%
dv = amax/vmax; % agent damping parameter
da = jmax/amax;
A = [0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 -dv 0 1 0;
    0 0 0 -dv 0 1;
    0 0 0 0 -da 0;
    0 0 0 0 0 -da];
x0 = xhist(1,:);
xsim = simulateSys_tripleint(x0,t,xd,yd,kp,kd,jmax,A,B,N);


% compute residual error
for j = 1:1:N
    diffX = xsim(:,6*j-5) - xhist(:,6*j-5);
    diffY = xsim(:,6*j-4) - xhist(:,6*j-4);
    res(j) = sum(sqrt((diffX).^2 + (diffY).^2))/length(diffX);
end

cost = sum(res)/N;



end