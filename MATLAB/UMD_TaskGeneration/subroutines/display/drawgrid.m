function drawgrid(xmin,numXpx,ymin,numYpx,dx)
xmax = xmin + numXpx*dx;
ymax = ymin + numYpx*dx;
mag = 0.6;
for i = 0:1:numXpx
    plot([xmin xmin]+[1 1]*dx*i, [ymin ymax],'-','Color',[1 1 1]*mag);
    hold on;
end
for i = 0:1:numYpx
    plot([xmin xmax], [ymin ymin]+[1 1]*dx*i,'-','Color',[1 1 1]*mag);
end
end