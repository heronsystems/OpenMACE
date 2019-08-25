function plotTaskTable(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel )

numRows = size(swarmWorldHist{1}.priorP,1);
numCols = size(swarmWorldHist{1}.priorP,2);
taskTableFigHand = figure;
imagesc(trueWorld.xcp,trueWorld.ycp, swarmWorldHist{1}.mutualInfoSurface); %,'AlphaData',abs(cellStateMat)./2);
set(gca,'YDir','Normal')
axis tight;
axis equal;
% agents = [1:1:swarmModel.N];
agents = 2;
% vectorize cell array data
pause;
for i = 1:1:length(swarmWorldHist)
    swarmWorld = swarmWorldHist{i};
    t(i) = swarmWorld.time;
    nodeDensityEstimate(i) = swarmWorld.nodeDensityEstimate;
    % compute entropy of entire search grid
    entropyOfCells = swarmWorld.entropyMat;
    totalEntropy(i) = swarmWorld.totalEntropy;
    % swarm state (x,y)
    m = 1;
    for j = agents
        swarmState = swarmStateHist{i};
        xk = [ swarmState.x(4*j-3); swarmState.x(4*j-2); swarmState.x(4*j-1); swarmState.x(4*j) ];
        swarmAgentX(i,j) = xk(1);
        swarmAgentY(i,j) = xk(2);
        swarmAgentXd(i,j) = swarmState.xd(j);
        swarmAgentYd(i,j) = swarmState.yd(j);
        figure(taskTableFigHand);
        subplot(length(agents), 1, m);
        imagesc(trueWorld.xcp,trueWorld.ycp, swarmWorld.mutualInfoSurface); %,'AlphaData',abs(cellStateMat)./2);
        set(gca,'YDir','Normal')
        hold on;
        for k = 1:1:length(swarmWorld.cellCenterOfMass(:,1))
            plot(swarmWorld.cellCenterOfMass(k,1), swarmWorld.cellCenterOfMass(k,2),'mo','linewidth',2);
%             valueString = sprintf('%3.1f',swarmWorld.taskTable(j,k));
%             text(swarmWorld.cellCenterOfMass(k,1)+0.5, swarmWorld.cellCenterOfMass(k,2),valueString,'FontSize',12);
        end
        plot(swarmAgentX(i,j),swarmAgentY(i,j),'c*','linewidth',2);
        plot([swarmAgentX(i,j) swarmAgentXd(i,j)],[ swarmAgentY(i,j) swarmAgentYd(i,j)],'rs-','linewidth',2);
        % plot path at horizon
        x = xk;
        u = [0 ; 0];
        pathAgent = [xk(1) xk(2)];
        taskLocation = [swarmState.xd(j) swarmState.yd(j)];
        for k = 1:round(swarmModel.planningHorizon/runParams.dt)
            u = [u waypointController(x,taskLocation(1),taskLocation(2),swarmModel.kp_wpt,swarmModel.kd_wpt,swarmModel.umax)];
            x = x + runParams.dt*(swarmModel.A*x + swarmModel.B*u(:,end));
            pathAgent = [pathAgent; x(1:2)'];
            % adaptive terminal time: if the agents is close to the taskLocation by
            % Rsense, then we terminate the process of computing trajectory and
            % effort
            if sqrt((x(1)-taskLocation(1))^2+(x(2)-taskLocation(2))^2) <= swarmModel.Rsense/10
                break;
            end
        end
        plot(pathAgent(:,1), pathAgent(:,2),'ro','linewidth',2);
        
        axis equal; axis tight;
        set(gca,'FontSize',16)
        m = m + 1;
    end
    hold off;
    % plot task table
end





% subplot(1,3,1)
% plot(t, numNodesDiscovered / numNodes,'linewidth',2);
% xlabel('Time (Sec.)');
% ylabel('Vertices Discovered (%)');
% set(gca,'FontSize',16)
% grid on;
% subplot(1,3,2)
% plot(t, pixelsExplored ./ (width*height),'linewidth',2);
% xlabel('Time (Sec.)');
% ylabel('Area Discovered (%)');
% set(gca,'FontSize',16)
% grid on;
% subplot(1,3,3)
% plot(t, numNodesInView./numNodes,'linewidth',2);
% xlabel('Time (Sec.)');
% ylabel('Vertices In View (%)');
% set(gca,'FontSize',16)
% grid on;


end