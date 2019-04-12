% reflect the vehicles for voronoi boundaries
MX = mirror(X,WORLD);

% calculate and plot voronoi
[VX,VY] = voronoi(MX(:,1),MX(:,2));
plot(VX,VY,'g-','linewidth',2);
[V,C] = voronoin(MX);
T = tworld(t1,t2,WORLD,TP);
[CM,M] = calcCM(X,V,C,-T,t1,t2);
M = max(12*M/max(M),6);
for jj=1:N,
    h = [h; plot(X(jj,1),X(jj,2),'ro','linewidth',2, ...
        'markersize',M(jj))];
end