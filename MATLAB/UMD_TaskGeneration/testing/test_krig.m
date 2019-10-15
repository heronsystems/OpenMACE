%
clear all; close all; clc;

curPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/MACE_DecentralizedRTA/MATLAB/UMD_TaskGeneration/testing';
rootPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/MACE_DecentralizedRTA/MATLAB/UMD_TaskGeneration/';
addpath(curPath);
addpath(rootPath);
updatePath(rootPath,curPath);

load('krigData_90sec')

% pre-process
swarmWorld = swarmWorldHist{end};
V = swarmWorld.V;
numBinsX = trueWorld.numBinsX;
numBinsY = trueWorld.numBinsY;
X = trueWorld.xx;
Y = trueWorld.yy;
xcp = trueWorld.xcp;
ycp = trueWorld.ycp;
cellStateMat = swarmWorld.cellStateMat;

% determine unexplored area
[rows, cols] = find(cellStateMat == 2);

% used for prior of target
numNodesExploredGraph = length(swarmWorld.log_likelihood_env);
numUnexploredCells = numBinsX*numBinsY - numNodesExploredGraph;
probAbsent = swarmModel.probAbsentPrior;
factor = (1 - probAbsent)/numUnexploredCells;


Vmeas = V;
% determine which cells have been found to contain nodes
k = 1;
measurements = [];
measurementsNode = [];
kn = 1;
for i = 1:1:numBinsY
    for j = 1:1:numBinsX
        if ( swarmWorld.cellStateMat(i,j) ~= 2 ) % cellStateMat(j,i) == 1 )
            measurements(k,1) = trueWorld.xcp(j);
            measurements(k,2) = trueWorld.ycp(i);
            measurements(k,3) = V(i,j);
            k = k + 1;
        else
            Vmeas(i,j) = NaN;
        end
        if ( swarmWorld.cellDetMat(i,j) == 1 )
            measurementsNode(kn,1) = trueWorld.xcp(j);
            measurementsNode(kn,2) = trueWorld.ycp(i);
            measurementsNode(kn,3) = 1-V(i,j);
            kn = kn + 1;
            Nprob(i,j) = 1-V(i,j);
        else
            Nprob(i,j) = NaN;
        end
    end
end

U = swarmWorld.U;
O = swarmWorld.O;

figure;
subplot(2,3,1)
imagesc(Vmeas,'AlphaData',~isnan(Vmeas)); colorbar; title('Data (All Explored Void/Nodes)'); set(gca,'Ydir','Normal'); axis equal;
axis tight;
% subplot(1,3,2)
% imagesc(V); colorbar; title('Prior V / No Krig'); set(gca,'Ydir','Normal'); axis equal;
% axis tight;


R = 10;
meanVal = 1-exploredAreaNodeDensity(cellStateMat);
forecast = simpleKrig(X,Y,measurements,meanVal,R);
forecast(forecast > 1) = 1;
forecast(forecast < 0) = 0;

subplot(2,3,2)
imagesc('XData',xcp,'YData',ycp,'CData',forecast); colorbar; title('Simple Krig'); set(gca,'Ydir','Normal'); axis equal;
axis tight;

% assign prior
for i = 1:1:length(rows)
    r = rows(i);
    c = cols(i);
    V(r,c) = forecast(r,c);
    % error check
    if ( V(r,c) < 0 || V(r,c) > 1)
        error('V(r,c) is not a valid probability');
    end
    O(r,c) = (1 - V(r,c))*factor;
    U(r,c) = 1 - O(r,c) - V(r,c);
end
H_C = cellStateEntropyVectorized(V, U, O);
subplot(2,3,5)
imagesc(H_C); colorbar; title('Entropy Simple Krig.'); set(gca,'Ydir','Normal'); axis equal;caxis([0 1]);
axis tight;

forecast = ordinaryKrig(X,Y,measurements,R);
forecast(forecast > 1) = 1;
forecast(forecast < 0) = 0;

subplot(2,3,3)
imagesc(forecast); colorbar; title('Ordinary Krig'); set(gca,'Ydir','Normal'); axis equal;
axis tight;

% assign prior
for i = 1:1:length(rows)
    r = rows(i);
    c = cols(i);
    V(r,c) = forecast(r,c);
    % error check
    if ( V(r,c) < 0 || V(r,c) > 1)
        error('V(r,c) is not a valid probability');
    end
    O(r,c) = (1 - V(r,c))*factor;
    U(r,c) = 1 - O(r,c) - V(r,c);
end
H_C = cellStateEntropyVectorized(V, U, O);
subplot(2,3,6)
imagesc(H_C); colorbar; title('Entropy Ordinary Krig.'); set(gca,'Ydir','Normal'); axis equal;caxis([0 1]);
axis tight;

%%
figure;
subplot(2,3,1)
imagesc(Nprob,'AlphaData',~isnan(Nprob)); colorbar; title('Data (Nodes Only)'); set(gca,'Ydir','Normal'); axis equal;
axis tight;
caxis([0 1]);
predictedNodeProbMat = krigInterp(X,Y,measurementsNode,R);
% scale the interpolated cellStateMat from floor to cieling
% the user defined maximum is the cieling
maxUnexploredPrior = 0.85;
predictedNodeProbMat = normalizeMatrix(predictedNodeProbMat, 1-meanVal, maxUnexploredPrior);
% make a final pass and mask explored empty cells (set prob to zero) and
% lift up known nodes (set prob to one)
predictedNodeProbMatMask = 1-V;
for i = 1:1:numBinsY
    for j = 1:1:numBinsX
        switch cellStateMat(i,j)
            case 2
                predictedNodeProbMatMask(i,j) = predictedNodeProbMat(i,j);
        end
    end
end

predictedVoidMat = 1-predictedNodeProbMat;
predictedVoidMatMask = 1-predictedNodeProbMatMask;

subplot(2,3,2)
imagesc(predictedVoidMat); colorbar; title('Node-Only Krig'); set(gca,'Ydir','Normal'); axis equal;
axis tight; caxis([0 1]);
subplot(2,3,3)
imagesc(predictedVoidMatMask); colorbar; title('Node-Only Krig -- Masked'); set(gca,'Ydir','Normal'); axis equal;
axis tight;



% assign prior
for i = 1:1:length(rows)
    r = rows(i);
    c = cols(i);
    V(r,c) = predictedVoidMat(r,c);
    % error check
    if ( V(r,c) < 0 || V(r,c) > 1)
        error('V(r,c) is not a valid probability');
    end
    O(r,c) = (1 - V(r,c))*factor;
    U(r,c) = 1 - O(r,c) - V(r,c);
end
H_C = cellStateEntropyVectorized(V, U, O);

subplot(2,3,6)
imagesc( H_C ); colorbar; title('Entropy'); set(gca,'Ydir','Normal'); axis equal;
caxis([0 1]);
axis tight;


% subplot(1,3,2)
% imagesc(V); colorbar; title('Prior V / No Krig'); set(gca,'Ydir','Normal'); axis equal;
% axis tight;



