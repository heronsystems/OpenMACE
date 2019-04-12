function [angdiff] = signedAngularDist(ang1, ang2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [angdiff] = signedAngularDist(ang1, ang2)
%
% Inputs
%   (ang1, ang2): two angles in radians (vector inputs admissible)
%
% Output
%   angdiff: the smallest distance between the two angles. Result is on the
%   interval [-pi, pi] with the sign representing direction. Positive 
%   result implies that the smallest angle from ang1 to ang2 is measured
%   CCW.
%
% A. Wolek, July 2018  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ang1 = mod(ang1,2*pi);
ang2 = mod(ang2,2*pi);
angdiff = ang2-ang1;
for i = 1:1:length(ang1)
    if ( angdiff(i) > pi )
        angdiff(i) = -(2*pi - angdiff(i));
    elseif ( angdiff(i) < -pi )
        angdiff(i) = (2*pi + angdiff(i));
    end
end
end 