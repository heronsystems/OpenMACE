function [cellDetMat, detectedCells, V, U, O] = detectNodes( cellDetMat , V, U, O, cellsInView, threshold )


detectedCells = [];
for i = 1:1:size(cellsInView,1)
    bx = cellsInView(i,1);
    by = cellsInView(i,2);
    if ( cellDetMat(by,bx) == 0 )
        voidLR = V(by,bx)/( U(by,bx) + O(by,bx) );
        nodeLR = 1/voidLR;
%         if ( voidLR >= threshold )
%             cellDetMat(by,bx) = -1;
%             V(by,bx) = 1;
%             U(by,bx) = 0;
%             O(by,bx) = 0;
    %else
        if (nodeLR >= threshold)
            cellDetMat(by,bx) = 1;
            V(by,bx) = 0;
            sum = U(by,bx) + O(by,bx);
            % normalize
            U(by,bx) = U(by,bx) / sum;
            O(by,bx) = O(by,bx) / sum;
            detectedCells = [detectedCells; bx by];
        end
    end
end

if (isempty(detectedCells) )
    disp('No Cells Detected');
end

end