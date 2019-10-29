function [angdiff] = angularDist(ang1, ang2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [angdiff] = angularDist(ang1, ang2)
%
% Inputs
%   (ang1, ang2): two angles [radians]
%
% Output
%   angdiff: the smallest distance between the two angles. Result is on the
%   interval [0, pi] and does not give the sign.
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ang1 = mod(ang1,2*pi);
ang2 = mod(ang2,2*pi);
angdiff = ang2-ang1;
for i = 1:1:length(ang1)
    if ( angdiff(i) > pi )
        angdiff(i) = 2*pi - angdiff(i);
    elseif ( -angdiff(i) > pi )
        angdiff(i) = 2*pi + angdiff(i);
    elseif ( angdiff(i) < 0 )
        angdiff(i) = -angdiff(i);
    end
end
end