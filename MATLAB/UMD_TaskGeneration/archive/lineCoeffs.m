function [a,b,c] = lineCoeffs(x1, y1, x2, y2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [a,b,c] = lineCoeffs(x1, y1, x2, y2)
%
% Inputs
%   (x1, y1) : first endpoint of line segment
%   (x2, y2) : second endpoint of line segment
%
% Output
%   (a,b,c) : coefficients assuming line eq. is : ax + by + c = 0
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% vertical line
if (x2 == x1)
    a = 1; b = 0; c = -x1;
% horizontal line     
elseif (y2 == y1)
    a = 0; b = 1; c = -y1;
% arbitrary line    
else
    a = -(y2-y1)/(x2-x1);
    b = 1;
    c = -y1 - a*x1;
end
