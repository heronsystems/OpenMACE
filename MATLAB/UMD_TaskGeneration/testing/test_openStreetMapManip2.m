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

%% remove some ways
plotOpenStreetMapLabels(fileName, refX, refY, boxlength, boxwidth, angle, dx)
removeList = [4,3,34,35,36,37,7,61,25];
%removeList = [];
%% plot occupancy graph
% clip

[nodesXY] = loadOpenStreetMapNodesFlex(fileName, refX, refY, boxlength, boxwidth, angle, dx, removeList);

% 
% % plot edges
% edgeFactor = 1;
% for i = 1:1:length(nodesXY)
%     % calculate distance to all other nodes
%     dvec = (nodesXY(i,:) - nodesXY);
%     d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
%     ind = find(d <= edgeFactor*dx);
%     if ( length(ind) >= 2 )
%         for j = 1:1:length(ind)
%             if ind(j) ~= i
%                 % draw edge
%                 plot( [nodesXY(i,1) nodesXY(ind(j),1)],[nodesXY(i,2) nodesXY(ind(j),2)],'k-')
%                 hold on;
%             end
%         end
%     end
% end
% plot(nodesXY(:,1), nodesXY(:,2),'ko','MarkerSize',4)
% xlim([xmin xmin+numXpx*dx]);
% ylim([ymin ymin+numYpx*dx]);
% set(gca,'FontSize',16)
% axis equal;
% axis tight;
% hold on;
% xlabel('X (m)')
% ylabel('Y (m)')
% title('Occupancy Graph')
