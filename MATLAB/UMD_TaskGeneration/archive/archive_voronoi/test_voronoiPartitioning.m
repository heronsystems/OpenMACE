clear all; close all; clc;

% load map
[runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_Randals();

% perform kriging
sig = 5.0;
measurements = [trueWorld.nodeXY ones(length(trueWorld.nodeXY),1)];

% % modify measurements to have a hole in the middle
addHoleFlag = 0;
if ( addHoleFlag )
   measurementsTemp = measurements;
   measurements = [];
   for i = 1:1:length(measurementsTemp)
      pt = trueWorld.nodeXY(i,:);
      circpt = [200 200];
      d = norm(pt-circpt);
      R = 100;
      if ( d >= R )
         measurements = [measurements; pt 1]; 
      end
   end
end

forecast = krigInterp(trueWorld.xx, trueWorld.yy , measurements ,sig);
figure;
imagesc('XData',trueWorld.xcp, 'YData', trueWorld.ycp,'CData', forecast);
set(gca,'YDir','Normal')
view(2)
axis equal;
colorbar;
hold on;

% partition with equal ;
N = 100;
stepSizeGain = 0.2;
percentTol = 0.01;
maxIters = 3000;
[voronoiVertices, voronoiCells, cellMass, cellCenterOfMass] = approxEqualMassVornoiPartition(trueWorld.xx,trueWorld.yy,forecast,N,stepSizeGain,percentTol,maxIters);
for i = 1:1:length(voronoiCells)
    ind = voronoiCells{i};
    ind = [ind ind(1)];
    plot(voronoiVertices(ind,1),voronoiVertices(ind,2),'mo-','linewidth',2);
    hold on;
end
plot(cellCenterOfMass(:,1), cellCenterOfMass(:,2),'m+','linewidth',2);
axis equal;
xlim([trueWorld.minX trueWorld.maxX])
ylim([trueWorld.minY trueWorld.maxY])
xlabel('X (m)')
ylabel('Y (m)')
set(gca,'FontSize',16)

figure;
%tot = sum(sum(forecast));
tot = sum(cellMass);
idealWeightPerCell = tot/N;
histogram(cellMass./idealWeightPerCell,10)
xlabel('Percent of Ideal Equal Weight')
ylabel('Number of Cells')
set(gca,'FontSize',16)


