function [xCM, yCM, cellMass] = capacityConstrainedVoronoiCM(xx,yy,zz,Ac, numCells)
for i = 1:1:numCells
    [rows, cols, ~] = find( Ac == i );
    numPts = length(rows);
    xsum = 0;
    ysum = 0;
    msum = 0;
    for j = 1:1:numPts
       row = rows(j);
       col = cols(j);
       xsum = xsum + zz(row,col)*xx(row,col);
       ysum = ysum + zz(row,col)*yy(row,col);
       msum = msum + zz(row,col);
    end
    xCM(i) = xsum / msum;
    yCM(i) = ysum / msum;
    cellMass(i) = msum;
end
end