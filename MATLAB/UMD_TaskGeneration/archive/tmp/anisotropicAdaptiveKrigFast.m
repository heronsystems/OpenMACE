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

function forecast = anisotropicAdaptiveKrigFast(X,Y,measurements, E, peaks, weights)
Nx = size(X,1);
Ny = size(X,2);
forecast = zeros(Nx,Ny);

if ( ~isempty(measurements) )
    
    % measurements are of form [binX binY measuredValue]
    bx = measurements(:,1);
    by = measurements(:,2);
    nm = length(measurements);
    
    % build the covariance matrix of measurements-to-measurements
    % weights are given by the edge histogram
    C = zeros(nm,nm);
    for i = 1:1:length(peaks)          
        C = C + weights(i)*E(by,bx,i);
    end
    
    % augment last row/col Eq. 5.25
    C(end+1,:) = ones(1,nm);
    C(:,end+1) = [ones(1,nm) 0]';
    Cinv = inv(C); % Eq. 5.41    
    z = measurements(:,3);
    
    numAngles = size(E,3);
    % go through each prediction point
    for i = 1:1:Nx
        for j = 1:1:Ny
            B = [];
            %delPts = [x(i)-xm, y(j)-ym];
            for k = 1:1:nm
                %delP = delPts(k,:);
                Btemp = 0;
                for l = 1:1:numAngles     
                    %distKernel0(l) = norm(D{l} *delP');
                    Btemp = Btemp +  weights(l)*E(i,j,l); %exp(-distKernel0(l));
                    B(k,1) = Btemp;
                end              
            end
            B = [B; 1];
            w = Cinv*B; % weights, Eq. 5.42
            mu = w(end);
            lam = w(1:end-1);
            forecast(j,i) = lam'*(z-mu)+mu;% linear estimate, Eq. 5.48
        end
    end
    
    forecast( forecast > 1 ) = 1;
    forecast( forecast < 0 ) = 0;
    
    
    
end