%
clear; close all; clc;
run('./updatePath.m')

% % right plot : manipulated map with edges
fileName = 'RandallsIsland_Big.osm';
refX = -350;
refY = 80;
angle = -37*pi/180;
boxlength = 500;
dx = 1;
buffer = 1;
R = 2;
f3Width = 26;
f3Length = (59+28);
f3Aspect = f3Width/f3Length;
f3LowerLeftCornerX = -59;
f3LowerLeftCornerY = -13;
perimX = [0 1 1 0 0]*f3Length+f3LowerLeftCornerX;
perimY = [1 1 0 0 1]*f3Width+f3LowerLeftCornerY;
numPts = 20;
[pole1x, pole1y] = generateCircle(0, 0, R, numPts);
[pole2x, pole2y] = generateCircle(-30, 0, R, numPts);


%% plot baseline map
[ways, LatRef, LongRef] = parseOpenStreetMap(fileName);

figure;
for i = 1:1:length(ways)
    plot(ways{i}(:,1), ways{i}(:,2),'o-')
    hold on;
end
set(gca,'FontSize',16)
axis equal;
axis tight;
hold on;
grid on;
boxwidth = boxlength*f3Aspect;
boxXY = boxCorners(refX,refY,boxlength,boxwidth,angle);
plot([boxXY(:,1); boxXY(1,1)],[boxXY(:,2); boxXY(1,2)],'k-','linewidth',2);
xlabel('East')
ylabel('North')
title('Open Street Map Data: Randall''s Island')

%% plot open street map clipped and transformed
% clip
scale = f3Length/boxlength;
shiftX = -refX;
shiftY = -refY;
waysmod = manipOpenStreetMap( ways, boxXY, -angle, scale, shiftX, shiftY );

% shift
figure;
xmin = f3LowerLeftCornerX;
ymin = f3LowerLeftCornerY;
numXpx = floor(f3Length/dx);
numYpx = floor(f3Width/dx);
drawgrid(xmin,numXpx,ymin,numYpx,dx)
xlim([xmin xmin+numXpx*dx]);
ylim([ymin ymin+numYpx*dx]);

hold on;
for i = 1:1:length(waysmod)
    waysmod{i} = waysmod{i} + [f3LowerLeftCornerX f3LowerLeftCornerY];
end
for i = 1:1:length(waysmod)
    plot(waysmod{i}(:,1), waysmod{i}(:,2),'o-')
    hold on;
end
set(gca,'FontSize',16)
axis equal;
axis tight;
hold on;
grid on;
xlim([xmin xmin+numXpx*dx]);
ylim([ymin ymin+numYpx*dx]);
xlabel('X (m)')
ylabel('Y (m)')
title('Open Street Map Data Transformed')
plot(perimX, perimY, 'k-','Linewidth',2);
plot(pole1x, pole1y, 'k-','Linewidth',2);
plot(pole2x, pole2y, 'k-','Linewidth',2);

%% plot occupancy graph
% compute F3 nodes 
nodesXY = loadOpenStreetMapNodes(fileName, refX, refY, boxlength, boxwidth, angle, dx, buffer);

figure;
% plot grid
drawgrid(xmin,numXpx,ymin,numYpx,dx)
hold on;
axis equal;

% plot perimeter, poles
plot(perimX, perimY, 'k-','Linewidth',2);
plot(pole1x, pole1y, 'k-','Linewidth',2);
plot(pole2x, pole2y, 'k-','Linewidth',2);
hold on;

% plot edges
edgeFactor = 1;
for i = 1:1:length(nodesXY)
    % calculate distance to all other nodes
    dvec = (nodesXY(i,:) - nodesXY);
    d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
    ind = find(d <= edgeFactor*dx);
    if ( length(ind) >= 2 )
        for j = 1:1:length(ind)
            if ind(j) ~= i
                % draw edge
                plot( [nodesXY(i,1) nodesXY(ind(j),1)],[nodesXY(i,2) nodesXY(ind(j),2)],'k-')
                hold on;
            end
        end
    end
end
plot(nodesXY(:,1), nodesXY(:,2),'ko','MarkerSize',4)
xlim([xmin xmin+numXpx*dx]);
ylim([ymin ymin+numYpx*dx]);
set(gca,'FontSize',16)
axis equal;
axis tight;
hold on;
xlabel('X (m)')
ylabel('Y (m)')
title('Occupancy Graph at F3')
% print(4,'-fillpage','-dpdf','discretizedF3_05m.pdf')