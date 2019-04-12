%%%%%%%%%%%%%%
function A = calcA(X,Tx,R0,WORLD,k);
n = length(X);
T = zeros(n,n);
G = zeros(n,n);
ORDER = -1;
mWORLD = mean(WORLD);
for ii=1:n, for jj=1:(ii-1),
    r = sqrt((X(ii,1)-X(jj,1))^2+(X(ii,2)-X(jj,2))^2);
    T(ii,jj) = (Tx(ii)-Tx(jj))/r;
    G(ii,jj) = mWORLD^(1-ORDER)*(r-R0)/r^(3-ORDER);
end, end
T = T-T';
if k(1)<0, T=max(T,0); end
T = T-diag(sum(T,2));
G = G+G';
G = G-diag(sum(G,2));
A = abs(k(1))*T+k(2)*G;