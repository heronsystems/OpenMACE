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

function forecast = anisotropicAdaptiveKrigLocal(X,Y,measurements,ax,ay, angles, weights,xcp, ycp)
Nx = size(X,1);
Ny = size(X,2);
forecast = zeros(Nx,Ny);

T = [1/ax 0; 0 1/ay];
if ( ~isempty(measurements) )
    
    % process measurements
    xm = xcp(measurements(:,1))';
    ym = ycp(measurements(:,2))';

    
%     [Xm,Ym] = meshgrid(xm,ym);        
%     delX = Xm - Xm';
%     delY = Ym - Ym';
%     distKernel1 = sqrt((delX/ax).^2 + (delY/ay).^2);
%     distKernel2 = sqrt((delX/ay).^2 + (delY/ax).^2);
   
%     tic
    
    % transform coordinates for kernel
    for i = 1:1:length(angles)       
        D{i} = T*[cos(angles(i)) sin(angles(i)); -sin(angles(i)) cos(angles(i))];
    end

    nm = length(measurements);
    
    for i = 1:1:nm
        for j = 1:1:nm
            delP = [xm(j) - xm(i), ym(j) - ym(i)];
            for k = 1:1:length(angles)       
                distKernel{k}(i,j) = norm(D{k} *delP');
                1;
            end            
        end
    end
    
    C = zeros(nm,nm);
    for i = 1:1:length(angles)       
        C = C + weights(i)*exp(-distKernel{i});
    end
    

    % augment last row/col Eq. 5.25
%    nm = length(xm);
    C(end+1,:) = ones(1,nm);
    C(:,end+1) = [ones(1,nm) 0]';
    
    Cinv = inv(C); % Eq. 5.41
    
    x = X(1,:)';
    y = Y(:,1);
    z = measurements(:,3);
       
    % weights
    for i = 1:1:Nx
        for j = 1:1:Ny
            B = [];
            % find measurements within numBins
            numBins = 5; 
            [indValidX] = find( (measurements(:,1) <= i+numBins) & (measurements(:,1) >= i-numBins) );
            [indValidY] = find( measurements(indValidX,2) <= j+numBins & measurements(indValidX,2) >= j-numBins );
            validM = indValidX(indValidY); 
            if (~isempty(validM) )
            delPts = [xcp(i)-xm(validM), ycp(j)-ym(validM)];
            for k = 1:1:length(validM)
                delP = delPts(k,:);
                Btemp = 0;
                for l = 1:1:length(angles)       
                    distKernel0(l) = norm(D{l} *delP');
                    Btemp = Btemp +  weights(l)*exp(-distKernel0(l));
                    B(k,1) = Btemp;
                end              
            end
            B = [B; 1];
            Cmod = C(validM, validM);
            nmv = numel(validM);
            Cmod(end+1,:) = ones(1,nmv);
            Cmod(:,end+1) = [ones(1,nmv) 0]';              
            w = inv(Cmod)*B; % weights, Eq. 5.42
            mu = w(end);
            lam = w(1:end-1);
            forecast(j,i) = lam'*(z(validM)-mu)+mu;% linear estimate, Eq. 5.48
            else
            forecast(j,i) = 0;    
            end
        end
    end
    
    forecast( forecast > 1 ) = 1;
    forecast( forecast < 0 ) = 0;
    
    
    
end