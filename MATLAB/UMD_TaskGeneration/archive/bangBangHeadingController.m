function [turnRateCmd] = bangBangHeadingController( h, hd, tol, umax )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [turnRateCmd] = bangBangHeadingController( h, hd, tol, uMax )
%
% This function computes a turn rate command the causes the vehicle to
% lazily move towards the  node requiring the smallest heading change.
%
% Inputs
%   h : (rad) current agent heading
%   hd : (rad) desired heading 
%   tol : (rad) deadband
%   umax : (rad/s) max turn rate 
%
% Output
%   turnRateCmd : computed using a bang-bang controller 
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hDiffSigned = signedAngularDist( h, hd );
if ( hDiffSigned > tol )
    turnRateCmd = umax;
elseif ( hDiffSigned < -tol )
    turnRateCmd = -umax;
else
    turnRateCmd = 0;
end
end