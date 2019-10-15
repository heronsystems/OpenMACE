%function forecast = krig(X,Y,measurements,sig)
%
% Description: Implements a linear kriging algorithm
%
% Input:
%   measurements : (M x 3) matrix, with each row (xval, yval, zval)
%   X : (Nrows x Ncols) grid of X values
%   Y : (Nrows x Ncols) grid of Y values
%   sigma : scalar parameter > 0 (effects blurring)
%
% Output:
%   forecast : (Nrows x Ncols) of interpolated zvals
%
% Notes:
%
% A prediction yhat is expressed as a linear function of measurements.
% yhat = sum_(i=1)^N w_i yi
%   where N: is the number of measurements y_i
%         w_i: are weights to be determined
%         with  w(i) = exp(-dist/sig)*sigma
%
% Artur Wolek, 2018
%
% Adapted from: forecast = krig(X,Y,measurements,sig)  Derek Paley, 2015

function forecast = krigInterp(X,Y,measurements,sig)
Nx = size(X,1);
Ny = size(X,2);
forecast = zeros(Nx,Ny);
if ( ~isempty(measurements) )
for i = 1:1:Nx
    for j = 1:1:Ny
        dx = measurements(:,1) - X(i,j);
        dy = measurements(:,2) - Y(i,j);
        dist = (dx.^2 + dy.^2).^(1/2);
        weights = exp(-dist./sig);
        forecast(i,j) = weights'*measurements(:,3);
    end
end
end

end