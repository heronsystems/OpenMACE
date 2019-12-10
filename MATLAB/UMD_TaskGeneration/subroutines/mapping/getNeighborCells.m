function [bxNeighbor, byNeighbor] = getNeighborCells( bx, by, numBinsX, numBinsY )

    % get all eight neighbors
    bxNeighbor(1) = bx-1;
    byNeighbor(1) = by-1;

    bxNeighbor(2) = bx-1;
    byNeighbor(2) = by;    
    
    bxNeighbor(3) = bx-1;
    byNeighbor(3) = by+1; 
    
    bxNeighbor(4) = bx;
    byNeighbor(4) = by+1;
    
    bxNeighbor(5) = bx+1;
    byNeighbor(5) = by+1;    

    bxNeighbor(6) = bx+1;
    byNeighbor(6) = by;

    bxNeighbor(7) = bx+1;
    byNeighbor(7) = by-1;
    
    bxNeighbor(8) = bx;
    byNeighbor(8) = by-1;    

    % check if any of them are 
    valid = [];
    for i = 1:1:8
        if ( byNeighbor(i) >= 1 && byNeighbor(i) <= numBinsY ...
             && bxNeighbor(i) >= 1 && bxNeighbor(i) <= numBinsX )
            valid = [valid i];
        end
    end
    bxNeighbor = bxNeighbor(valid);
    byNeighbor = byNeighbor(valid);

end