%function forecast = simpleKrig(X,Y,measurements,mean,R)
%
% Description: Implements a simple kriging algorithm
%
% Input:
%   measurements : (M x 3) matrix, with each row (xval, yval, zval)
%   X : (Nrows x Ncols) grid of X values
%   Y : (Nrows x Ncols) grid of Y values
%   mean : a user-defined mean, the forecast reaches this mean at points
%   away from all the measurements (e.g., a distance >> R)
%   R : a shape parameter (similar to radius of influence)
%
% Output:
%   forecast : (Nrows x Ncols) of interpolated zvals
%
% Ref. Zekai Sen, "Spatial Modeling Principles in Earth Sciences", 2009.
%
% Artur Wolek, 2018

function forecast = simpleKrig(X,Y,measurements,mu,R)
Nx = size(X,1);
Ny = size(X,2);
forecast = zeros(Nx,Ny);
if ( ~isempty(measurements) )
    
    % standardize measurements
    sig = std(measurements(:,3));
    measurements(:,3) = (measurements(:,3)-mu)/sig;
    xm = measurements(:,1);
    ym = measurements(:,2);
    [Xm,Ym] = meshgrid(xm,ym);

    % compute the distance matrix
    D = ((Xm-Xm').^2 + (Ym-Ym').^2).^(1/2);

    % semivariogram that is exponential
    rho = exp(-D/R); % Eq. 5.36, see Fig. 5.15
    
    Cinv = inv(rho); % Eq. 5.41
    
    x = X(1,:)';
    y = Y(:,1);
    z = measurements(:,3);

for i = 1:1:Nx
    for j = 1:1:Ny    
        d = ((x(i)-xm).^2 +(y(j)-ym).^2).^(1/2);
        B = exp(-d/R);
        w = Cinv*B; % weights, Eq. 5.42
        forecast(j,i) = w'*z;% linear estimate, Eq. 5.38 
    end
end
    
% convert back to 
forecast = sig.^2*(forecast)+mu;

end