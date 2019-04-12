%
clear all; close all; clc;

curPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/MACE_DecentralizedRTA/MATLAB/UMD_TaskGeneration/testing';
rootPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/MACE_DecentralizedRTA/MATLAB/UMD_TaskGeneration/';
addpath(curPath);
addpath(rootPath);
updatePath(rootPath,curPath);

[runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_cityblocks();



% derived world model parameters
trueWorld = loadEnvironment(trueWorld, targetModel);
[ trueWorld.Q ] = stateTransitionMatrix ( trueWorld.G_tss, targetModel.probStopping, targetModel.m , targetModel.d );

Ns = numnodes(trueWorld.G_tss);

% plot target state space
figure;
h = plot(trueWorld.G_tss);
% create labels for target state space nodes
% "p > c" indicates state corresponds to target arrived at c from p
for i = 1:1:Ns
    labels{i} = [num2str(trueWorld.G_tss.Nodes.prevNode(i)) '>' num2str(trueWorld.G_tss.Nodes.curNode(i))];
end
labelnode(h,[1:1:Ns],labels);
axis equal;

% plot matrix
imagesc(trueWorld.Q);
colorbar;
xlabel('State Index')
ylabel('State Index')
set(gca,'FontSize',16);
axis equal;

% add weight to edges (for plotting)
for i = 1:1:Ns
    [eid, nid] = outedges(trueWorld.G_tss,i);
    if ( ~isempty(eid) )
        for j = 1:1:length(eid)
            trueWorld.G_tss.Edges.Weight( eid(j) ) = trueWorld.Q(i, nid(j) );
        end
    end
end



% simulate motion update
likelihood = zeros(numnodes(trueWorld.G_tss),1);

% assign a random state
randomState = 15; %randi([1 Ns]);
likelihood(randomState) = 1;

k = 1;
dt = 1;
T = 10;
figure;
N = numnodes(trueWorld.G_env);





fig = figure('rend','painters','pos',[100 100 1000 400]);
subplot(1,2,1)
p1 = plot(trueWorld.G_env,'XData',trueWorld.G_env.Nodes.x,'YData',trueWorld.G_env.Nodes.y);
set(gca,'FontSize',16)
xlabel('X(m)')
ylabel('Y(m)')
title('Environment Graph')
colorbar;
set(gcf,'color','white')
set(gca, 'color', [0.8 0.8 0.8])
axis equal;

subplot(1,2,2)
%p2 = plot(G_tss,'EdgeLabel',G_tss.Edges.Weight)
p2 = plot(trueWorld.G_tss)
set(gca,'FontSize',16)
title('Target State Graph')
set(gcf,'color','white')
set(gca, 'color', [0.8 0.8 0.8])
%labelnode(p2,[1:1:Ns],labels);
axis equal;
axis off;

figure(fig)
pause

for t = 1:dt:T
    
    subplot(1,2,1)
    titleString = ['Environment Graph, k = ' num2str(t)];
    title(titleString)
    sum(likelihood)
    likelihood_flat = trueWorld.Mc'*likelihood;
    
    p1.MarkerSize = 6;
    p1.NodeCData = likelihood_flat;
    colorbar;
    
    p2.NodeCData = likelihood;
    p2.MarkerSize = 6;
    
    likelihood = trueWorld.Q'*likelihood;
    k = k + 1;
    drawnow;
    pause;
end


