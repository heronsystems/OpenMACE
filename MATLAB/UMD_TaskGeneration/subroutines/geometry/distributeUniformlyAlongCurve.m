function [xv, yv, prevPt] = distributeUniformlyAlongCurve(N,x,y)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: [xv, yv, prevPt] = distributeUniformlyAlongCurve(N,x,y)
%
% Inputs
%   N : number of points to distribute
%   (x,y) : vectors defining the curve
%
% Output
%   (xv, yv) : vectors defining the distributed points 
%   prevPt : vector indicating the index of (x,y) that immediately
%            precedes each distributed point
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% compute length of each segment along polygon
dx = diff(x);
dy = diff(y);
ds = (dx.^2+dy.^2).^(1/2);
% compute cumulative arc-length 
s(1) = 0;
for i = 2:1:length(ds)+1
   s(i) = ds(i-1) + s(i-1); 
end
sTotal = s(end);
% compute uniformly distributed normalized arc-length
sNorm = [0 : 1/N : (N-1)/N]; 
% shift by random amount 
sNorm = mod(sNorm + rand(), 1);

ind = [1:1:length(x)];
% allocate
xv = zeros(N,1);
yv = zeros(N,1);
prevPt = zeros(N,1);
% linear interpolation
for i = 1:1:length(sNorm)
    xv(i) = interp1(s,x,sNorm(i)*sTotal);
    yv(i) = interp1(s,y,sNorm(i)*sTotal);
    prevPt(i) = interp1(s,ind,sNorm(i)*sTotal,'previous');
end

end