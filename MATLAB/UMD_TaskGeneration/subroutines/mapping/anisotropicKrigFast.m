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

function forecast = anisotropicKrigFast(X,Y,measurements,ax,ay)
disp('anisotropic');
tic;
Nx = size(X,1);
Ny = size(X,2);
forecast = zeros(Nx,Ny);

T = [1/ax 0; 0 1/ay];
if ( ~isempty(measurements) )
    
    % sigma
    sig = std(measurements(:,3));
    
    % process measurements
    xm = measurements(:,1);
    ym = measurements(:,2);
    [Xm,Ym] = meshgrid(xm,ym);
   
%     % transform coordinates for kernel 1 
%     th1=0;
%     R1 = [cos(th1) sin(th1); -sin(th1) cos(th1)];    
%     D1 = T*R1;
%     
%     th2=pi/2;
%     R2 = [cos(th2) sin(th2); -sin(th2) cos(th2)];
%     D2 = T*R2; 
   
    delX = Xm - Xm';
    delY = Ym - Ym';    
    distKernel1 = sqrt((delX/ax).^2 + (delY/ay).^2);
    distKernel2 = sqrt((delX/ay).^2 + (delY/ax).^2);
    
%     for i = 1:1:length(measurements)
%        for j = 1:1:length(measurements)
%            delP = [xm(j) - xm(i), ym(j) - ym(i)];
%            distKernel1(i,j) = norm(D1*delP');
%            distKernel2(i,j) = norm(D2*delP');
%        end
%     end

%     
%     % compute the distance matrix
%     D = ((Xm-Xm').^2 + (Ym-Ym').^2).^(1/2);
%     
    % semivariogram that is exponential
    C = exp(-distKernel1) + exp(-distKernel2); % Eq. 5.25, see Fig. 5.25
    toc;
    1;
    
    % augment last row/col Eq. 5.25
    nm = length(xm);
    C(end+1,:) = ones(1,nm);
    C(:,end+1) = [ones(1,nm) 0]';
    toc;
    
    Cinv = inv(C); % Eq. 5.41
    
    
    
    
    x = X(1,:)';
    y = Y(:,1);
    z = measurements(:,3);
    
    twoAxisAnisotropicKrigMex(x,y,Cinv,measurements,ax,ay)
    
    % weights
    for i = 1:1:Nx
        for j = 1:1:Ny
            B = [];
            delPts = [x(i)-xm, y(j)-ym];
            for k = 1:1:length(delPts)
                distKernel20 = norm(D1*delPts(k,:)');
                distKernel10 = norm(D2*delPts(k,:)');
                B(k,1) =  exp(-distKernel10) + exp(-distKernel20);
            end
                        
            B = [B; 1];
            w = Cinv*B; % weights, Eq. 5.42
            mu = w(end);
            lam = w(1:end-1);
            forecast(j,i) = lam'*(z-mu)+mu;% linear estimate, Eq. 5.48 
        end
    end
    
    
end