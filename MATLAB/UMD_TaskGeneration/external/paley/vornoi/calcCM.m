%%%%%%%%%%%%%%%%%%%%
% calculate center of mass of voronoi cells
function [CM,M] = calcCM(X,V,C,T,t1,t2)
n = length(X);
M = zeros(n,1);
CM = zeros(n,2);
for ii=1:n,
    VC = V(C{ii},:);
    in = find(inpolygon(t1,t2,VC(:,1),VC(:,2)));
    M(ii) = sum(sum(T(in)));
    if abs(M(ii))<1e-9, continue, end
    CM(ii,1) = sum(sum(T(in).*t1(in)))/M(ii);
    CM(ii,2) = sum(sum(T(in).*t2(in)))/M(ii);   
end