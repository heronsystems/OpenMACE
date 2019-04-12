function [xpts, ypts, xinit, yinit] = lawnmowerRectangle(xmin,xmax, ymin, ymax, initCorner, R)
% initCorner 1 - lower left, 2 - upper left, 3 - upper right, 4 - lower
% right

% dimensions
Lx = xmax - xmin;
Ly = ymax - ymin;

% determine number of swaths
numSwaths = floor( (Ly/2) / (2*R) ) + 1;
if (numSwaths*2*R > Ly/2)
    leftover = numSwaths*2*R - (Ly/2);    
    offset = leftover/(numSwaths);
    dely = 2*R - offset;
elseif (numSwaths*2*R == Ly/2)
    dely = 2*R;
else
    error('initializeSwarmState : numSwaths*2*R > Ly/2')
end
% determine direction of survey
%scale = 0.9;
%R = R*scale;

delx = Lx/2;
if initCorner == 1 % lower left
    dirx = 1;
    diry = 1;
    startX = xmin;    
    startY = ymin+dely/2;
    xinit = xmin;
    yinit = ymin+dely/2;
elseif initCorner == 2 % upper left
    dirx = 1;
    diry = -1;
    startX = xmin;
    startY = ymax-dely/2;
    xinit = xmin;
    yinit = ymax-dely/2;
elseif initCorner == 3 % upper right
    dirx = -1;
    diry = -1;
    startX = xmax;
    startY = ymax-dely/2;
    xinit = xmax;
    yinit = ymax-dely/2;
elseif initCorner == 4 % lower right
    dirx = -1;
    diry = 1;
    startX = xmax;
    startY = ymin+dely/2;
    xinit = xmax;
    yinit = ymin+dely/2;
else
    error('lawnmowerRectangle: initCorner input invalid')
end

k = 1;
dir = dirx;
xpts = zeros(numSwaths*2,1);
ypts = zeros(numSwaths*2,1);
for i = 1:1:numSwaths
    if ( dir == dirx)
        % boundary point
        xpts(k) = startX;
        ypts(k) = startY + diry*(i-1)*dely;
        k = k + 1;
        % center point
        xpts(k) = startX + delx*dirx;
        ypts(k) = startY + diry*(i-1)*dely;
        k = k + 1;
    elseif ( dir == -dirx)
        % center point
        xpts(k) = startX + delx*dirx;
        ypts(k) = startY + diry*(i-1)*dely;
        k = k + 1;
        % boundary point 
        xpts(k) = startX;
        ypts(k) = startY + diry*(i-1)*dely;
        k = k + 1;        
    end
    dir = -dir;
end

end



