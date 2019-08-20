function plotOpenStreetMapLabels(fileName, refX, refY, boxlength, boxwidth, angle, dx)

% parse baseline map
[ways, LatRef, LongRef] = parseOpenStreetMap(fileName);

% generate box
boxXY = boxCorners(refX,refY,boxlength,boxwidth,angle);

% clip, shift to origin, and scale
shiftX = -refX;
shiftY = -refY;
waysmod = manipOpenStreetMap( ways, boxXY, -angle, 1, shiftX, shiftY );

% define grid
xmin = 0;
ymin = 0;
numX = floor(boxlength/dx);
numY = floor(boxwidth/dx);

% plot ways with labels
for i = 1:1:length(waysmod)
plot(waysmod{i}(:,1), waysmod{i}(:,2) );
hold on;
text(waysmod{i}(floor(end/2),1), waysmod{i}(floor(end/2),2)+rand(), num2str(i) );
end

drawgrid(xmin,numX,ymin,numY,dx);
hold on;
xlim([xmin xmin+numX*dx]);
ylim([ymin ymin+numY*dx]);

end