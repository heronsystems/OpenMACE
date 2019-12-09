function [value, isterminal, direction] = wptReachedEvent(t, x, params)

dist = sqrt((x(1)-params.xd)^2+(x(2)-params.yd)^2);
if dist <= params.Rsense/10
    value = 0;   
else
    value = dist;
end
isterminal = 1;   % Stop the integration
direction  = 0;

end