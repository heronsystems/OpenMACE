function [bxNeighbor, byNeighbor] = getNeighborCells( bx, by, numBinsX, numBinsY )

    % get all eight neighbors
    bxNeighbor(1) = bx-1;
    byNeighbor(1) = by;

    bxNeighbor(2) = bx;
    byNeighbor(2) = by+1;

    bxNeighbor(3) = bx+1;
    byNeighbor(3) = by;

    bxNeighbor(4) = bx;
    byNeighbor(4) = by-1;

    % check if any of them are 
    valid = [];
    for i = 1:1:4
        if ( byNeighbor(i) >= 1 && byNeighbor(i) <= numBinsY ...
             && bxNeighbor(i) >= 1 && bxNeighbor(i) <= numBinsX )
            valid = [valid i];
        end
    end
    bxNeighbor = bxNeighbor(valid);
    byNeighbor = byNeighbor(valid);

end