clear; close all; clc;
rootPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration';
curPath = pwd;
addpath(rootPath);
addpath(curPath);
cd(rootPath)
updatePath;
cd(curPath);

load NYC.mat

% draw control points
for i = 1:1:trueWorld.numBinsX
    for j = 1:1:trueWorld.numBinsY
        plot(trueWorld.xcp(i),trueWorld.ycp(j),'ko');
        hold on;
    end
end

% draw grid
drawgrid(trueWorld.minX-trueWorld.binWidth/2,trueWorld.numBinsX,trueWorld.minY-trueWorld.binWidth/2,trueWorld.numBinsY,trueWorld.binWidth);
axis equal;
axis tight;

% draw nodes
for i = 1:1:length(trueWorld.nodeXY)
    plot(trueWorld.nodeXY(i,1), trueWorld.nodeXY(i,2),'r*')
end

% draw connection from bin to node
for i = 1:1:trueWorld.numBinsX
    for j = 1:1:trueWorld.numBinsY
        n = trueWorld.bin2NodeID(j,i); % recall this convention is because rows correspond to y-axis
        if ( n > 0 )
            plot([trueWorld.xcp(i) trueWorld.nodeXY(n,1)],[trueWorld.ycp(j) trueWorld.nodeXY(n,2)],'o-')
        end
        hold on;
    end
end

figure;
% draw nodes
for i = 1:1:length(trueWorld.nodeXY)
    plot(trueWorld.nodeXY(i,1), trueWorld.nodeXY(i,2),'ko')
    text(trueWorld.nodeXY(i,1)+0.1, trueWorld.nodeXY(i,2)+0.1,num2str(i))
    hold on;
end
% plot edges
edgeFactor = trueWorld.binWidth;
for i = 1:1:length(trueWorld.nodeXY)
        % calculate distance to all other nodes
        dvec = (trueWorld.nodeXY(i,:) - trueWorld.nodeXY);
        d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
        ind = find(d <= edgeFactor);
        if ( length(ind) >= 2 )
            for j = 1:1:length(ind)
                if ind(j) ~= i
                    % draw edge
                    plot( [trueWorld.nodeXY(i,1) trueWorld.nodeXY(ind(j),1)],[trueWorld.nodeXY(i,2) trueWorld.nodeXY(ind(j),2)],'k-')
                    hold on;
                end
            end
        end
end
axis equal;

figure;
imagesc(trueWorld.bin2NodeID)
colorbar
