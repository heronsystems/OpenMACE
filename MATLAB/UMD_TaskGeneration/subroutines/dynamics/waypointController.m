function u = waypointController(x,xd,yd,kp,kd,umax)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: u = waypointController(x,xd,yd,kp,kd,umax)
%
% Inputs
%   x : state of an agent modeled with double-integrator dynamics
%       [x, y, xdot, ydot]'
%   (xd,yd) : desired waypoint
%   (kp,kd) : PD gains
%   umax : saturating on magnitude of acceleration 
%
% Output
%   u : 2 x 1 vector of accelerations [ax ay]'
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% PD control 
u(1,1) = -kp*(x(1) - xd) - kd*x(3);
u(2,1) = -kp*(x(2) - yd) - kd*x(4);
% saturate
if ( norm(u) > umax )
    u = u/norm(u)*umax;
end
end