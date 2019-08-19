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

function forecast = anisotropicAdaptiveKrigFast2(X,Y,measurements, ax,ay, anglesRad, weights, xcp, ycp)
Nx = size(X,1);
Ny = size(X,2);
forecast = zeros(Nx,Ny);

if ( ~isempty(measurements) )
    
    % process measurements from bins to control pts
    xm = xcp(measurements(:,1))';
    ym = ycp(measurements(:,2))';
    nm = size(measurements,1);
    
    T = [1/ax 0; 0 1/ay]; % scaling
    % transform coordinates for kernel
    for i = 1:1:length(anglesRad)       
        M{i} = T*[cos(anglesRad(i)) sin(anglesRad(i)); -sin(anglesRad(i)) cos(anglesRad(i))];
    end
    
%     N = length(xcp)*length(ycp); % total number of control pts 
    numAngles = length(anglesRad);        
    E = zeros(nm,nm,numAngles); % E is a (N,N,numAngles) tensor
%     pts = zeros(numel(X),2); % here we flatten meshgrid into vector 
%     pts(:,1) = reshape(X,N,1);
%     pts(:,2) = reshape(Y,N,1);
%     % the number of unique pairs of points is : N^2 
     [xxm,yym] = meshgrid(xm, ym); 
%     % del is a matrix defining the delta cartesian difference between the 
%     % pairs of points
     del(1,:) = reshape(xxm - xxm', nm*nm,1);
     del(2,:) = reshape(yym - yym', nm*nm,1);
% %     % error checking 
%     maxVal = norm([min(xcp) min(ycp)]-[max(xcp) max(ycp)]);
%     maxVal = max([maxVal, ax*maxVal, ay*maxVal]);
    % main loop
    for i = 1:1:length(anglesRad)
        % M combines the scaling + rotation
        %M = T*[cos(anglesRad(i)) sin(anglesRad(i)); -sin(anglesRad(i)) cos(anglesRad(i))];
        % multiple M by the direction vectors in del 
        vals = M{i}*del;
        % store the exponential of all the scaled distances into E
        E(:,:,i) = reshape(exp(-vecnorm(vals)),[nm nm]);
        % make symmetric to avoid round-off error    
        % E(:,:,i) = (Etemp + Etemp')/2; 
    end
%     if ( max(max(E)) > maxVal )
%        error('Error: max E value '); 
%     end


    
%     % measurements are of form [binX binY measuredValue]
%     bx = measurements(:,1);
%     by = measurements(:,2);
    
    
    % build the covariance matrix of measurements-to-measurements
    % weights are given by the edge histogram
    C = zeros(nm,nm);
    for i = 1:1:length(numAngles)          
        C = C + weights(i)*E(:,:,i);
    end
    
    % augment last row/col Eq. 5.25
    C(end+1,:) = ones(1,nm);
    C(:,end+1) = [ones(1,nm) 0]';
    Cinv = inv(C); % Eq. 5.41    
    z = measurements(:,3);
    
    % go through each prediction point
    for i = 1:1:Nx
        for j = 1:1:Ny
             B = zeros(nm,1);
            delPts = [xcp(i)-xm, ycp(j)-ym];
            for l = 1:1:numAngles 
            vals = M{l}*delPts';
            % store the exponential of all the scaled distances into E
            B = B + weights(l)*reshape(exp(-vecnorm(vals)),[nm 1]);
            end
%             for k = 1:1:nm
%                 delP = delPts(k,:);
%                 Btemp = 0;
%                 for l = 1:1:numAngles     
%                     distKernel0(l) = norm(M{l} *delP');
%                     Btemp = Btemp +  weights(l)*exp(-distKernel0(l));
%                     B(k,1) = Btemp;
%                 end              
%             end
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