function [amod,bmod] = shrinkInterval(a,b,val)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [amod,bmod] = shrinkInterval(a,b,val)
%
% Inputs
%   (a,b) a range of values with b > a 
%   val : desired amount to shrink interval (from each end)

% Output
%   (amod,bmod) is the range (a,b) narrowed by 2*val
%    i.e., amod = a + val, bmod = b - val
%
% A. Wolek, July 2018       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ( b - a > 2*val )
   amod = a + val;
   bmod = b - val;
else
    disp('shrinkInterval: Warning cannot shrink using requested values, returning input.');
    amod = a;
    bmod = b;
end


end