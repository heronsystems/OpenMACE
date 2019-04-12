function [turnRateCmd] = followNetworkTurnRateCmd(x,y,h,nodesX,nodesY,hDiffTol,umax)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [turnRateCmd] = followNetworkTurnRateCmd(x,y,h,nodesX,nodesY,params)
%
% This function computes a turn rate command the causes the vehicle to
% lazily move towards the  node requiring the smallest heading change.
%
% Inputs
%   (x,y,h) : state of an agent modeled as a dubins vehicle
%   (nodeX, nodeY) : a subset of nodes on a graph (in close proximity)
%   hdiff : (rad) heading deadband
%   umax : (rad/s) max turn rate 
%
% Output
%   turnRateCmd : computed using a bang-bang controller 
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% compute heading to each node
hNodes = atan2( nodesY - y , nodesX - x );

% compute offset from current heading
hDiff = angularDist(h*ones(size(hNodes)),hNodes);

% find most closesly alligned node
[hDiffMinVal, hDiffMinInd] = min(hDiff);

% bang-bang control input with deadband to achieve this heading
if ( ~isempty(hDiffMinInd) )
    turnRateCmd = bangBangHeadingController( h, hNodes(hDiffMinInd), hDiffTol, umax );
else
    turnRateCmd = 0;
end

end