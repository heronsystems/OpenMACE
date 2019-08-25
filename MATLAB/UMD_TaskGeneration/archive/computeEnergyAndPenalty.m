function [costAndPenalty] = computeEnergyAndPenalty(state,taskLocation,swarmModel,simParams)
%
% Description: compute the sum of energy and penalty for one agent to reach
% the target
%
% Input:
%   state : (1 x 4) vector, represents [x y xdot ydot] of the agent
%   taskLocation : (1 x 2) vector of target location
%   swarmModel : a struct to hold parameters
%   simParams: a struct to hold parameters
%
% Output:
%   costAndPenalty : scalar value
%
% Notes: the factor is assigned to the energy term to enable energy and
% penalty has the same scale
%
% Sheng Cheng, 2018

% initialization
costAndPenalty = 0;
u = [];
x = state';

% first propogate the state to compute all future control actions and the 
% terminal state
for k = 1:round(swarmModel.Tsamp/simParams.dt)
    u = [u waypointController(x,taskLocation(1),taskLocation(2),swarmModel.kp_wpt,swarmModel.kd_wpt,swarmModel.umax)];
    x = x + simParams.dt*(swarmModel.A*x + swarmModel.B*u(:,end));
end
% now u stores all the control actions
% x is the terminal state at a future time swarmModel.wptChangePeriod/simParams.dt

% then compute energy = the sum of squares of all control actions
% NOTE: we use a factor of 15 to balance the scale of cost and penalty
costAndPenalty = costAndPenalty + 15*simParams.dt*(u(1,:)*u(1,:)' + u(2,:)*u(2,:)');

% then add to energy the penalty = distance between terminal state and
% target
costAndPenalty = costAndPenalty + sqrt((x(1:2)'-taskLocation)*(x(1:2)-taskLocation'));
