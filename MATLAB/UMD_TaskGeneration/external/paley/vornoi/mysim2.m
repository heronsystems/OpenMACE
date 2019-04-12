%function [XI,X] = mysim(N)
%function [XI,X] = mysim(XI)
function [XI,X] = mysim(args)
global END

% initialize
initsim

% scalar weights (xdot = Ka*A+Kg*G)
Ka = 1;
Kg = .2;

% equilibrium range
R0 = 20;

% asymmetric action force?
ASYM = 0; 

% iterate over time
for ii=1:TIME,
    
    % delete the figure handles
    try, delete(h), catch, end
    
    % calc the scalar measurement vector
    Tx = calcTx(X,T);
    
    % calc the measurement interaction matrix
    [Ax,Ay] = calcA(X,Tx,ASYM);
    
    % calc the range interaction matrix
    [Gx,Gy] = calcG(X,R0);
    
    % calc the x component of velocity
    u = Ka/WORLD*Ax+Kg*WORLD^2*Gx;
    
    % calc the y component of velocity
    v = Ka/WORLD*Ay+Kg*WORLD^2*Gy;
    
    % plot positions
    h = plot(X(:,1),X(:,2),'ro');
    
    % plot velocity vector
    g = quiver(X(:,1),X(:,2),u,v,0);
    
    % keep the quiver handle for deleting
    if ~TRACKS, h = [h; g]; end
    
    % update the figure
    drawnow
    
    % update the positions
    X(:,1) = X(:,1)+u;
    X(:,2) = X(:,2)+v;
    
    % check runtime flag
    if END, break, end
end

% reflect the vehicles for voronoi boundaries
MX = mirror(X,WORLD);

% calculate and plot voronoi
[VX,VY] = voronoi(MX(:,1),MX(:,2));
h = plot(VX,VY,'g-');
    

%%%%%%%%%%%%%%
% the range interaction matrices
% xdot = (-r+r0)/r^3
function [Gx,Gy] = calcG(X,R0)
n = length(X);
Ux = zeros(n,n);
Uy = zeros(n,n);
for ii=1:n, for jj=1:(ii-1),
    Ux(ii,jj) = X(ii,1)-X(jj,1);   
    Uy(ii,jj) = X(ii,2)-X(jj,2);
end, end
Ux = Ux-Ux';
Uy = Uy-Uy';
U = sqrt(Ux.^2+Uy.^2)+diag(ones(n,1));
M = (-U+R0)./U.^3/n;
Gx = sum(M.*Ux./U,2);
Gy = sum(M.*Uy./U,2);


%%%%%%%%%%%%%%
% the measurement interaction matrix
% for N=3, A = [-T12-T13 T12 T13; -T12 -T21-T23 T23; -T12 -T23 -T31-T32]
function [Ax,Ay] = calcA(X,Tx,ASYM);
n = length(Tx);
A = zeros(n,n);
for ii=1:n, for jj=1:(ii-1),
    A(ii,jj) = Tx(jj)-Tx(ii);        
end, end
A = A-A';
A = A+diag(sum(A));
Ax = A*X(:,1);
Ay = A*X(:,2);
if ASYM,
    [Ux,Uy] = calcU(X);
    ii = find(Ax.*Ux+Ay.*Uy<0);
    Ax(ii) = 0;
    Ay(ii) = 0;
end


%%%%%%%%%%%%%%%
% the measurement vector
% Tx = T(X)
function Tx = calcTx(X,T);
n = length(X);
Tx = zeros(n,1);
X = floor(X);
for ii=1:n,
    try, Tx(ii) = T(X(ii,2),X(ii,1)); catch, end    
end


%%%%%%%%%%%%%%
% Unit vectors
% for N=3, U = [-2 1 1; 1 -2 1; 1 1 -2]
function [Ux,Uy] = calcU(X);
n = length(X);
U = ones(n,n);
U = U-n*diag(ones(n,1));
Ux = U*X(:,1);
Uy = U*X(:,2);
