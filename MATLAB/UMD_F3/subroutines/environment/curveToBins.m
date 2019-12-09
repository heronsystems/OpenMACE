function [bins] = curveToBins(x,y,dx,numX, numY)
% bins are numbered from top left of image, going to the right 
% assume first bin is
boxX = dx*numX;
boxY = dx*numY;
for i = 1:1:length(x)
if ( x(i) >= 0 && x(i) <= boxX && y(i) >= 0 && y(i) <= boxY)            
    % get bin
    if (x(i) == boxX)
        bx = numX; % on RHS boundary of box
    else
        bx = floor(x(i)/dx)+1;
    end
    if (y(i) == boxY)
        by = numY; % on top boundary of box
    else
        by = floor(y(i)/dx)+1;
    end
    binNum = bx + (by-1)*numY;
    bins(i,:) = [bx by binNum];
else
    bins(i,:) = [NaN NaN NaN];
end
end

end