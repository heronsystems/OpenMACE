%function forecastMat = recursiveKrigInterp(forecastMat, X, Y, measurements, sig)
%
% Description: Implements a linear kriging algorithm
%
% Input:
%   forecast : (Nrows x Ncols) forecast
%   measurements : (M x 3) matrix, with each row (xval, yval, zval)
%                  These are *new* measurements
%   X : (Nrows x Ncols) grid of X values
%   Y : (Nrows x Ncols) grid of Y values
%   sigma : scalar
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
function forecastMat = recursiveKrigInterp(forecastMat, X, Y, measurements, sig)
M = size(measurements,1);
Nx = size(X,1);
Ny = size(X,2);
for m = 1:1:M
    for i = 1:1:Nx
        for j = 1:1:Ny
            dx = measurements(m,1) - X(i,j);
            dy = measurements(m,2) - Y(i,j);
            dist = (dx.^2 + dy.^2).^(1/2);
            weights = exp(-dist./sig);
            forecastMat(i,j) = forecastMat(i,j) + weights'*measurements(m,3);
        end
    end
end

end