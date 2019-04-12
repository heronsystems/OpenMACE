%
clear; close all; clc;
run('../updatePath.m')

% % right plot : manipulated map with edges
fileName = 'RandallsIsland_Big.osm';
refX = -300;
refY = -200;
angle = 0*pi/180;
boxlength = 400;
boxwidth = 400;
dx = 5;
buffer = 1;


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
boxXY = boxCorners(refX,refY,boxlength,boxwidth,angle);
plot([boxXY(:,1); boxXY(1,1)],[boxXY(:,2); boxXY(1,2)],'k-','linewidth',2);
xlabel('East')
ylabel('North')
title('Open Street Map Data: Randall''s Island')


%% plot open street map clipped and transformed

figure;
% plot grid
xmin = 0;
ymin = 0;
numXpx = floor(boxlength/dx);
numYpx = floor(boxwidth/dx);
drawgrid(xmin,numXpx,ymin,numYpx,dx)


% clip
scale = 1;
shiftX = -refX;
shiftY = -refY;
waysmod = manipOpenStreetMap( ways, boxXY, -angle, scale, shiftX, shiftY );

hold on;
for i = 1:1:length(waysmod)
    waysmod{i} = waysmod{i};
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

xlabel('X (m)')
ylabel('Y (m)')
xlim([min(boxXY(:,1)) max(boxXY(:,1))]-refX);
ylim([min(boxXY(:,2)) max(boxXY(:,2))]-refY);
title('Open Street Map Data Transformed')


%% plot occupancy graph
% clip

[nodesXY] = osmToNodes(fileName, refX, refY, boxlength, boxwidth, angle, dx, buffer);

figure;
% plot grid
xmin = 0;
ymin = 0;
numXpx = floor(boxlength/dx);
numYpx = floor(boxwidth/dx);
drawgrid(xmin,numXpx,ymin,numYpx,dx)
xlim([xmin xmin+numXpx*dx]);
ylim([ymin ymin+numYpx*dx]);

hold on;
axis equal;

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
title('Occupancy Graph')
