%function forecast = ordinaryKrig(X,Y,measurements,R)
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

function forecast = ordinaryKrig(X,Y,measurements,R)
Nx = size(X,1);
Ny = size(X,2);
forecast = zeros(Nx,Ny);
if ( ~isempty(measurements) )
    
    % sigma
    sig = std(measurements(:,3));
    
    % process measurements
    xm = measurements(:,1);
    ym = measurements(:,2);
    [Xm,Ym] = meshgrid(xm,ym);
    
    % compute the distance matrix
    D = ((Xm-Xm').^2 + (Ym-Ym').^2).^(1/2);
    
    % semivariogram that is exponential
    C = exp(-D/R); % Eq. 5.25, see Fig. 5.25
    
    % augment last row/col Eq. 5.25
    nm = length(xm);
    C(end+1,:) = ones(1,nm);
    C(:,end+1) = [ones(1,nm) 0]';
    
    Cinv = inv(C); % Eq. 5.41
    
    x = X(1,:)';
    y = Y(:,1);
    z = measurements(:,3);
    % weights
    for i = 1:1:Nx
        for j = 1:1:Ny
            d = ((x(i)-xm).^2 +(y(j)-ym).^2).^(1/2);
            B = [exp(-d/R); 1];
            w = Cinv*B; % weights, Eq. 5.42
            mu = w(end);
            lam = w(1:end-1);
            forecast(j,i) = lam'*(z-mu)+mu;% linear estimate, Eq. 5.48 
        end
    end
    
    
end