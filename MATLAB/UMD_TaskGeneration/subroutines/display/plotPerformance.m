function plotPerformance(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )

numRows = size(swarmWorldHist{1}.U,1);
numCols = size(swarmWorldHist{1}.U,2);
% vectorize cell array data
for i = 1:1:length(swarmWorldHist)
    t(i) = swarmWorldHist{i}.time;
    %nodeDensityEstimate(i) = swarmWorldHist{i}.nodeDensityEstimate;
    % compute entropy of entire search grid
    entropyOfCells = swarmWorldHist{i}.entropyMat;
    totalEntropy(i) = swarmWorldHist{i}.totalEntropy;
    % compute the percentage of discovered nodes
    discoveredNodePercentage(i) = 100*numnodes(swarmWorldHist{i}.exploredGraph)/numnodes(trueWorld.G_env);
    % swarm state (x,y)
    for j = 1:1:swarmModel.N
        swarmState = swarmStateHist{i};
        xk = [ swarmState.x(4*j-3); swarmState.x(4*j-2); swarmState.x(4*j-1); swarmState.x(4*j) ];
        swarmAgentX(i,j) = xk(1);
        swarmAgentY(i,j) = xk(2);
        swarmAgentXd(i,j) = swarmState.xd(j);
        swarmAgentYd(i,j) = swarmState.yd(j);
        
    end
end

% % number of cells explored with time
% figure;
% plot(t, swarmWorldHist{end}.mapPercentage,'linewidth',2);
% xlabel('Time (sec.)')
% ylabel('Percent Coverage')
% grid on;
% set(gca,'FontSize',16)
% 



% % node density
% figure;
% plot(t, nodeDensityEstimate,'linewidth',2);
% xlabel('Time (sec.)')
% ylabel('Node Density Estimate')
% grid on;
% set(gca,'FontSize',16)

% % entropy 
% maxEntropy = log2(3)*numRows*numCols/1000;
% fprintf('Max Entropy For Map is %3.1f (kilobits) \n', maxEntropy );
% figure;
% plot(t, totalEntropy/1000,'linewidth',2);
% ylim([0 maxEntropy/2])
% xlabel('Time (sec.)')
% ylabel('Total Entropy (kilobits)')
% grid on;
% set(gca,'FontSize',16)

% plot path of agents
figure;
cellStateMat = swarmWorldHist{end}.cellStateMat;
imagesc(trueWorld.xcp,trueWorld.ycp, cellStateMat); %,'AlphaData',abs(cellStateMat)./2);
set(gca,'YDir','Normal')
hold on;
axis equal;
axis tight;
for i = 1:1:swarmModel.N
    x = swarmAgentX(:,i);
    y = swarmAgentY(:,i);
    plot(x,y,'linewidth',3);
    hold on;
    xlabel('X')
    ylabel('Y')
    grid on;
    set(gca,'FontSize',16)
end

% plot actual vs desired waypoints
figure;
for i = 1:1:swarmModel.N    
    subplot(swarmModel.N,2,2*i);
    plot(t,swarmAgentX(:,i),'linewidth',2);    
    hold on;
    plot(t,swarmAgentXd(:,i),'+','linewidth',0.5);     
    xlabel('Time (sec.)')
    ylabel('X position');
    grid on;
    set(gca,'FontSize',16)
    
    subplot(swarmModel.N,2,2*i-1);
    plot(t,swarmAgentY(:,i),'linewidth',2);    
    hold on;
    plot(t,swarmAgentYd(:,i),'+','linewidth',0.5);   
    xlabel('Time (sec.)')
    ylabel('Y position');
    grid on;
    set(gca,'FontSize',16)    
end



swarmWorld = swarmWorldHist{end};
figure;
subplot(3,1,1)
imagesc(trueWorld.xcp,trueWorld.ycp,swarmWorld.V);
hold on;
set(gca,'YDir','Normal')
set(gca,'FontSize',16)
title('Prob {C = Void}')
axis equal; axis tight; colorbar; caxis([0 1]);
subplot(3,1,2)
imagesc(trueWorld.xcp,trueWorld.ycp,swarmWorld.U);
hold on;
set(gca,'YDir','Normal')
set(gca,'FontSize',16)
title('Prob {C = Unoccupied}')
axis equal; axis tight; colorbar; caxis([0 1]);
subplot(3,1,3) 
imagesc(trueWorld.xcp,trueWorld.ycp,swarmWorld.O);
hold on;
set(gca,'YDir','Normal')
set(gca,'FontSize',16)
title('Prob {C = Occupied}')
axis equal; axis tight; colorbar; caxis([0 1]);

figure;
subplot(2,1,1)
imagesc(trueWorld.xcp,trueWorld.ycp,swarmWorld.mutualInfoSurface);
hold on;
set(gca,'YDir','Normal')
set(gca,'FontSize',16)
title('Mutual Info')
ylim([0 1])
axis equal; axis tight; colorbar;
caxis([0 1])
subplot(2,1,2)
imagesc(trueWorld.xcp,trueWorld.ycp,swarmWorld.entropyMat);
hold on;
set(gca,'YDir','Normal')
set(gca,'FontSize',16)
title('entropyMat')
caxis([0 1])
axis equal; axis tight; colorbar;

% percentage of explored nodes
figure;
plot(t, discoveredNodePercentage,'linewidth',2);
xlabel('Time (sec.)')
ylabel('Percentage of discovered nodes')
grid on;
set(gca,'FontSize',16)

end