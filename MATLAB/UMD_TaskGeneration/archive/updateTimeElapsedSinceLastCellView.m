function timeElapsedMat = updateTimeElapsedSinceLastCellView(timeElapsedMat, cellsInView, dt)
timeElapsedMat = timeElapsedMat + dt;
for i = 1:1:size(cellsInView,1)
    bx = cellsInView(i,1);
    by = cellsInView(i,2);    
    timeElapsedMat(by,bx) = 0; % reset time if cell is in view
end
end