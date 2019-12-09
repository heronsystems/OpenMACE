function [numNodesDiscovered, numNodesInView, pixelsExplored] = calculatePerformance(t,x,xt,G,discoveredNodesHist,nodesInViewHist,params)
for k = 1:1:length(discoveredNodesHist)
    numNodesDiscovered(k) = length(discoveredNodesHist{k});
    numNodesInView(k) = length(nodesInViewHist{k});
end


imageMatHist = zeros(params.height,params.width);
pixelsExplored = zeros(length(t),1);
for m = 2:1:length(t)
    pixelsExplored(m) = pixelsExplored(m-1);
    for i = 1:1:params.N
        xi = [ x(m,3*i-2); x(m,3*i-1); x(m,3*i)];
        for j = 1:1:params.height % first index relates to y coordinate            
            yLoc = (params.maxYpixel - j);
            for k = 1:1:params.width % second index relates to x coord
                if ( imageMatHist(j,k) == 0 ) % check if 
                    xLoc = params.minXpixel + k;
                    d2 = ( xi(1) - xLoc )^2 + ( xi(2) - yLoc )^2;
                    if ( d2 <= params.Rsense^2 )
                        pixelsExplored(m) = pixelsExplored(m) + 1;
                        imageMatHist(j,k) = 1;
                    end
                end
            end
        end
    end
end
end