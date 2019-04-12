function explorationMat = convertCellStateMatToExplorationMat(cellStateMat)
explorationMat = zeros(size(cellStateMat));
for i = 1:1:size(cellStateMat,1)
    for j = 1:1:size(cellStateMat,2)
        if ( cellStateMat(i,j) == 1 || cellStateMat(i,j) == 0)
            explorationMat(i,j) = 0;
        elseif ( cellStateMat(i,j) == 2 )
            explortationMat(i,j) = 1;
        end
    end
end
end