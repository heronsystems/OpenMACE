function [value,isterminal,direction] = wallHitEvent(t,x,params)
N = length(x)/3;
for i = 1:1:N
    xi = [ x(3*i-2); x(3*i-1); x(3*i) ];
    [hitStatus, bounceDir, minDist] = hitsWall(xi,params.xpoly,params.ypoly,params.tol,params.bounceType);
    d(i) = minDist;
end
value = min(d);     % Detect height = 0
isterminal = 1;   % Stop the integration
direction = -1;   % Negative direction only
end