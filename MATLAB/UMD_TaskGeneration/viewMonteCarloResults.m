%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A Monte Carlo Engine for main_taskGeneration.m
% S. Cheng, A. Wolek, August 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
format compact;

updatePath;

% 1) first run 'analysis' on a new data set to generate an intermediate
% mat file (lengthy process)
% 2) then run 'plot' to quickly view/manipulate analyzed data
processingType = 'analysis'; % options are: 'analysis' or 'plot'

% user inputs:
algRange = 1:3; % index for algorithm
agentInitRange = 1:50; % index for generated scenes (agent initial location and target behavior)
mapRange = 1:3; % index for maps


totalTime=  tic;
for initialFormationID = 1:1:length(agentInitRange) % for target motions
    for mapID = 1:1:length(mapRange)
        for algorithmID = 1:1:length(algRange) %  for algorithm
            if strcmp(processingType,'analysis')
                fprintf('Analyzing algorithm %d (trial %d) \n', algorithmID, initialFormationID);
                str = ['./monteCarloRuns/MonteCarlo_Algorithm' num2str(algorithmID) '_InitialFormation' num2str(initialFormationID) '_mapID' num2str(mapID) '_TargetMotion' num2str(initialFormationID) '.mat'];
                load(str,'swarmWorldHist','trueWorld');
                disp('finished loading')
                
                % store relevant detection data
                % determine: detectionValid, detectionTime
                detectionFlag = swarmWorldHist{end}.targetDetectedFlag;
                if ( detectionFlag )
                    detectionValid = swarmWorldHist{end}.targetDetectionValid;
                    if ( detectionValid )
                        detectionTime = swarmWorldHist{end}.timeAtDetection;
                    else
                        detectionTime = NaN; %n/a
                    end
                else
                    detectionValid = NaN; % n/a
                    detectionTime = NaN; % n/a
                end
                
                % vectorize cell array data
                numRows = size(swarmWorldHist{1}.V,1);
                numCols = size(swarmWorldHist{1}.V,2);
                for i = 1:1:length(swarmWorldHist)
                    t(i) = swarmWorldHist{i}.time;
                    % compute entropy of entire search grid
                    entropyHist{i} = swarmWorldHist{i}.entropyMat;
                    totalEntropy(i) = swarmWorldHist{i}.totalEntropy;
                    
                    %
                    cellStateHist{i} = swarmWorldHist{i}.cellStateMat;
                    cellDetHist{i} = swarmWorldHist{i}.cellDetMat;
                    numViews(i) = swarmWorldHist{i}.numViews;
                    % compute the percentage of discovered nodes
                    discoveredNodePercentage(i) = 100*numnodes(swarmWorldHist{i}.exploredGraph)/numnodes(trueWorld.G_env);
                end
                mapPercentage = swarmWorldHist{end}.mapPercentage;
                
                % pack into alg/trial structure so it is all in one place
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.detectionFlag = detectionFlag;
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.detectionValid = detectionValid;
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.detectionTime = detectionTime;
                
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.cellDetHist = cellDetHist;
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.cellStateHist = cellStateHist;
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.numViews = numViews;
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.entropyHist = entropyHist;
                
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.t = t;
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.totalEntropy = totalEntropy;
                alg{algorithmID}.map{mapID}.trial{initialFormationID}.discoveredNodePercentage = discoveredNodePercentage;
                
                clearvars -except totalTime algRange agentInitRange mapRange processingType algorithmID mapID initialFormationID MonteCarloSwitch detection detectionValid detectionTime alg;
            end
            
        end
    end
end
fprintf('Total time is %f\n',toc(totalTime));
if strcmp(processingType,'analysis')
    save('MonteCarloData_processed.mat')
end


if strcmp(processingType,'plot')
    load('MonteCarloData_processed.mat');
    missionTimeMin = 5;
    % determine maximum number of pts
    maxPts = 0;
    numTrials = length(agentInitRange);
    for i = algRange
        for j = mapRange
            for k = agentInitRange
                numPts = length(alg{i}.map{j}.trial{k}.t);
                if ( maxPts < numPts )
                    maxPts = numPts;
                end
            end
        end
    end
    
    % visual inspection: plot all data
    for i = algRange
        for j = mapRange
            for k = agentInitRange
                figure(10)
                plot(alg{i}.map{j}.trial{k}.discoveredNodePercentage,'o-','linewidth',2)
                hold on;
            end
        end
    end
    
    % create matrix of data for each alg (trials x time)
    for i = algRange
        alg{i}.totalEntropyMat = NaN*ones(numTrials,maxPts);
        alg{i}.discoveredNodePercentageMat = NaN*ones(numTrials,maxPts);
        for j = 1:1:numTrials
            for m = mapRange
                numPts = length(alg{i}.map{m}.trial{j}.t);
                for k = 1:1:numPts % total target motions
                    alg{i}.totalEntropyMat(j+(m-1)*numTrials,k) = alg{i}.map{m}.trial{j}.totalEntropy(k);
                    alg{i}.discoveredNodePercentageMat(j+(m-1)*numTrials,k) = alg{i}.map{m}.trial{j}.discoveredNodePercentage(k);
                end
            end
        end
    end
    
    
    %% Fig 1: Nodes vs Time
    
    fs = 10;
    
    % set lawnmower colors
    mapB = brewermap(4,'Blues');
    mapG = brewermap(4,'Greens');
    mapP = brewermap(4,'Purples');
    font = 'Arial';
    %figh = figure('DefaultTextFontName', font, 'DefaultAxesFontName', font);
    alpha = 0.5;
    maxPts = length(alg{1}.map{1}.trial{1}.t);
    timeRange = [1:1:maxPts];
    hold on;
    x = timeRange./(missionTimeMin*60);
    
    % plot random motion
    data = alg{3}.discoveredNodePercentageMat;
    dataMean = meanOmitNaN(data,1);
    dataStdev = stdevOmitNaN(data,1);
    fig=figure(1);
    hold on;
    h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapB(3,:),'EdgeColor','none');
    p1 = plot(x,dataMean,'Color',mapB(4,:),'linewidth',1.5);
    set(h1,'FaceAlpha',alpha);
    axis tight;
    
    % plot lawnmower
    data = alg{2}.discoveredNodePercentageMat;
    dataMean = meanOmitNaN(data,1);
    dataStdev = stdevOmitNaN(data,1);
    hold on;
    h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapG(3,:),'EdgeColor','none');
    p2 = plot(x,dataMean,'Color',mapG(4,:),'linewidth',1.5);
    set(h1,'FaceAlpha',alpha);
    axis tight;
    
    % plot mutual Info
    data = alg{1}.discoveredNodePercentageMat;
    dataMean = meanOmitNaN(data,1);
    dataStdev = stdevOmitNaN(data,1);
    h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapP(3,:),'EdgeColor','none');
    p3 = plot(x,dataMean,'Color',mapP(4,:),'linewidth',1.5);
    hold on;
    set(h1,'FaceAlpha',alpha);
    axis tight;
    grid on;
    grid minor;
    hlg = legend([p3 p1 p2],{'Mutual Info','Random','Lawnmower'},'Location','SE');
    hlg.FontSize=10;
    ylim([0 100])
    xticks([0 0.2 0.4 0.6 0.8 1]);
    xlim([0 1])
    xlabel('Normalized Time')
    ylabel('Nodes Detected (%)')
    set(gca,'FontName','Arial','FontSize',fs)
    %title(labels{i})
    set(gcf,'Color','w')
    box on;
    axis square;
    set(gcf,'units','inches','position',[5,5,4,3])
    
    %% Fig 2: Target Det. vs Time
    
    %load('./monteCarloRuns/MonteCarlo_Algorithm1_InitialFormation1_mapID1_TargetMotion1.mat','trueWorld')
    [~,~, trueWorld, ~,~] = loadParams_osm();
    
    % scalar data is transformed into cell vector ( numAlgs x numTrials )
    for i = algRange
        numValidAlg = 0;
        numDetAlg = 0;
        for j = agentInitRange % total target motions
            for m = mapRange
                fprintf('Processing alg %d trial %d \n', i, j);
                % entropy
                entropyMat(i,j,m) = alg{i}.map{m}.trial{j}.totalEntropy(end);
                entropyMidwayMat(i,j,m) = alg{i}.map{m}.trial{j}.totalEntropy(floor(end/2));
                % det
                detectionFlagMat(i,j,m) = alg{i}.map{m}.trial{j}.detectionFlag;
                detectionValidMat(i,j,m) = alg{i}.map{m}.trial{j}.detectionValid;
                if ( detectionFlagMat(i,j,m)==1 && detectionValidMat(i,j,m) == 1 )
                    detectionTimeMat(i,j,m) = alg{i}.map{m}.trial{j}.detectionTime;
                    numValidAlg = numValidAlg + 1;
                    
                elseif ( detectionFlagMat(i,j,m)==1 && detectionValidMat(i,j,m) == 0 )
                    fprintf('Alg %d, Map %m, Trial %m: \n',i,m,j);
                    disp('Warning: Detection made that was invalid!');
                    detectionTimeMat(i,j,m) = NaN;
                else
                    detectionTimeMat(i,j,m) = NaN;
                end
                if ( detectionFlagMat(i,j,m)==1 )
                    numDetAlg = numDetAlg + 1;
                end
                % time
                for k = 1:1:length(alg{i}.map{m}.trial{j}.cellDetHist)
                    numNodesTotal = 0;
                    numNodesValid = 0;
                    for l = 1:1:trueWorld.numBinsX
                        for n = 1:1:trueWorld.numBinsY
                            % calculate number of valid nodes and total nodes
                            % detected
                            if( alg{i}.map{m}.trial{j}.cellDetHist{k}(l,n) == 1 )
                                numNodesTotal = numNodesTotal + 1;
%                                 if ( trueWorld.bin2NodeID(l,n) ~= 0)
%                                     numNodesValid = numNodesValid + 1;
%                                 end
                            end
                            % calculate the true entropy
                        end
                    end
                    % record entropy
                end
                percentNodesTotalMat(i,j,m) = numNodesTotal/trueWorld.numNodes;
                percentNodesValidMat(i,j,m) = numNodesValid/trueWorld.numNodes;
            end
            numValid(i,m) = numValidAlg;
            numDet(i,m) = numDetAlg;
        end
    end
    
    % Disp
    for i = algRange        
        fprintf('Alg %i had %d/%d valid detections \n',i, sum(numValid(i,:)), sum(numDet(i,:)));        
    end
    detTime_rm = [];
    detTime_mi = [];
    detTime_lm = [];
    
    
    
    alg_rm = 3;
    alg_mi = 1;
    alg_lm = 2;
    for m = mapRange
        for i = agentInitRange
            if ( detectionValidMat(alg_rm,i,m) == 1 )
                detTime_rm =  [detTime_rm detectionTimeMat(alg_rm,i,m)];
                if ( isnan(detTime_rm(end)) )
                   1; 
                end
            end
            if ( detectionValidMat(alg_mi,i,m) == 1 )
                detTime_mi =  [detTime_mi detectionTimeMat(alg_mi,i,m)];
                if ( isnan(detTime_mi(end)) )
                   1; 
                end                
            end
            if ( detectionValidMat(alg_lm,i,m) == 1 )
                detTime_lm =  [detTime_lm detectionTimeMat(alg_lm,i,m)];
                if ( isnan(detTime_lm(end)) )
                   1; 
                end                
            end
        end
    end
    
    
    % sort
    detTime_rm = sort(detTime_rm);
    detTime_lm = sort(detTime_lm);
    detTime_mi = sort(detTime_mi);
    
    numTrials = length(agentInitRange)*length(mapRange);
    
    % plot
    figure(2);
    %subplot(3,3,j);
    p1 = plot(detTime_rm/(missionTimeMin*60),[1:1:length(detTime_rm)]./numTrials*100,'-','Color',mapB(4,:),'linewidth',1.5);
    hold on;
    p2 = plot(detTime_lm/(missionTimeMin*60),[1:1:length(detTime_lm)]./numTrials*100,'-','Color',mapG(4,:),'linewidth',1.5);
    p3 = plot(detTime_mi/(missionTimeMin*60),[1:1:length(detTime_mi)]./numTrials*100,'-','Color',mapP(4,:),'linewidth',1.5);
    xlabel('Normalized Time')
    ylabel('Targets Detected (%)')
    set(gca,'FontName','Arial','FontSize',10)
    grid on;
    grid minor;
    set(gcf,'Color','w')
    xlim([0 1]);
    %ylim([0 100]);
    xticks([0 0.2 0.4 0.6 0.8 1]);
    %title(labels{j})
    if ( j == 1 )
        hlg = legend([p3 p1 p2], {'Mutual Info','Random','Lawnmower'},'Location','NW');
    end
    hlg.FontSize=10;
    
    axis square;
    set(gcf,'units','inches','position',[5,5,4,3])
    
    %% Fig. 3: nodes vs targets
    
    % get random motion data
    data = alg{ 3 }.discoveredNodePercentageMat;
    dataNodeMean_rm = meanOmitNaN(data,1);
    dataNodeStdev_rm = stdevOmitNaN(data,1);
    clear data;
    % get lm data
    data = alg{ 2 }.discoveredNodePercentageMat;
    dataNodeMean_lm = meanOmitNaN(data,1);
    dataNodeStdev_lm = stdevOmitNaN(data,1);
    clear data;
    % get
    data = alg{ 1 }.discoveredNodePercentageMat;
    dataNodeMean_mi = meanOmitNaN(data,1);
    dataNodeStdev_mi = stdevOmitNaN(data,1);
    clear data;
    
    % augment detection times with a zero
    detTime_rm = [ 0 detTime_rm ];
    detTime_lm = [ 0 detTime_lm ];
    detTime_mi = [ 0 detTime_mi ];
    
    % 
    
    detPercent_rm = [0:1:length(detTime_rm)-1]./numTrials;
    detPercent_lm = [0:1:length(detTime_lm)-1]./numTrials;
    detPercent_mi = [0:1:length(detTime_mi)-1]./numTrials;
    
    % make unique
    [detTime_rm, ind] = unique(detTime_rm,'last');
    detPercent_rm = detPercent_rm(ind);
    
    [detTime_lm, ind] = unique(detTime_lm,'last');
    detPercent_lm = detPercent_lm(ind);
    
    [detTime_mi, ind] = unique(detTime_mi,'last');
    detPercent_mi = detPercent_mi(ind);
    
    % need to interpolate to same number of points
    timeRange = [1:1:length(alg{1}.discoveredNodePercentageMat)];
    method = 'nearest';
    detTarget_rm_interp = interp1(detTime_rm, detPercent_rm, timeRange,method)*100;
    detTarget_lm_interp = interp1(detTime_lm, detPercent_lm, timeRange,method)*100;
    detTarget_mi_interp = interp1(detTime_mi, detPercent_mi, timeRange,method)*100;
    
    % init
    indNan = isnan(detTarget_rm_interp);
    detTarget_rm_interp(indNan) = [];
    dataNodeMean_rm(indNan) = [];
    dataNodeStdev_rm(indNan) = [];
    %timeRange_rm(indNan) = [];
    
    % random
    figure(3)
    [detTarget_rm_interp_fill_first,ind_first] = unique(detTarget_rm_interp,'first');
    [detTarget_rm_interp_fill_last,ind_last] = unique(detTarget_rm_interp,'last');
    dataNodeMean_rm_fill_first = dataNodeMean_rm(ind_first);
    dataNodeStdev_rm_fill_first = dataNodeStdev_rm(ind_first);
    dataNodeMean_rm_fill_last = dataNodeMean_rm(ind_last);
    dataNodeStdev_rm_fill_last = dataNodeStdev_rm(ind_last);
    xfill = [dataNodeMean_rm_fill_first-dataNodeStdev_rm_fill_first fliplr(dataNodeMean_rm_fill_last+dataNodeStdev_rm_fill_last)];
    yfill = [detTarget_rm_interp_fill_first fliplr(detTarget_rm_interp_fill_last)];
    
    figure(3);
    h1 = fill(xfill,yfill,mapB(3,:),'EdgeColor','none');
    set(h1,'FaceAlpha',alpha);
    p1 = plot(dataNodeMean_rm,detTarget_rm_interp,'-','Color',mapB(4,:),'linewidth',1.5);
    hold on;
    
    figure(4);
    hold on;
    h1b = fill(yfill,xfill,mapB(3,:),'EdgeColor','none');
    set(h1b,'FaceAlpha',alpha);
    p1b = plot(detTarget_rm_interp,dataNodeMean_rm,'-','Color',mapB(4,:),'linewidth',1.5);
    
    
    % lawnomwer
    indNan = isnan(detTarget_lm_interp);
    detTarget_lm_interp(indNan) = [];
    dataNodeMean_lm(indNan) = [];
    dataNodeStdev_lm(indNan) = [];
    %timeRange_lm(indNan) = [];
    [detTarget_lm_interp_fill_first,ind_first] = unique(detTarget_lm_interp,'first');
    [detTarget_lm_interp_fill_last,ind_last] = unique(detTarget_lm_interp,'last');
    dataNodeMean_lm_fill_first = dataNodeMean_lm(ind_first);
    dataNodeStdev_lm_fill_first = dataNodeStdev_lm(ind_first);
    dataNodeMean_lm_fill_last = dataNodeMean_lm(ind_last);
    dataNodeStdev_lm_fill_last = dataNodeStdev_lm(ind_last);
    xfill = [dataNodeMean_lm_fill_first-dataNodeStdev_lm_fill_first fliplr(dataNodeMean_lm_fill_last+dataNodeStdev_lm_fill_last)];
    yfill = [detTarget_lm_interp_fill_first fliplr(detTarget_lm_interp_fill_last)];
    
    figure(3);
    h2 = fill(xfill,yfill,mapG(3,:),'EdgeColor','none');
    set(h2,'FaceAlpha',alpha);
    p2 = plot(dataNodeMean_lm,detTarget_lm_interp,'-','Color',mapG(4,:),'linewidth',1.5);
    hold on;
    
    figure(4);
    h1b = fill(yfill,xfill,mapG(3,:),'EdgeColor','none');
    set(h1b,'FaceAlpha',alpha);
    p1b = plot(detTarget_lm_interp,dataNodeMean_lm,'-','Color',mapG(4,:),'linewidth',1.5);
    
    % mutual info
    indNan = isnan(detTarget_mi_interp);
    detTarget_mi_interp(indNan) = [];
    dataNodeMean_mi(indNan) = [];
    dataNodeStdev_mi(indNan) = [];
    %timeRange_mi(indNan) = [];
    [detTarget_mi_interp_fill_first,ind_first] = unique(detTarget_mi_interp,'first');
    [detTarget_mi_interp_fill_last,ind_last] = unique(detTarget_mi_interp,'last');
    dataNodeMean_mi_fill_first = dataNodeMean_mi(ind_first);
    dataNodeStdev_mi_fill_first = dataNodeStdev_mi(ind_first);
    dataNodeMean_mi_fill_last = dataNodeMean_mi(ind_last);
    dataNodeStdev_mi_fill_last = dataNodeStdev_mi(ind_last);
    xfill = [dataNodeMean_mi_fill_first-dataNodeStdev_mi_fill_first fliplr(dataNodeMean_mi_fill_last+dataNodeStdev_mi_fill_last)];
    yfill = [detTarget_mi_interp_fill_first fliplr(detTarget_mi_interp_fill_last)];
    figure(3);
    h3 = fill(xfill,yfill,mapP(3,:),'EdgeColor','none');
    set(h3,'FaceAlpha',alpha);
    p3 = plot(dataNodeMean_mi,detTarget_mi_interp,'-','Color',mapP(4,:),'linewidth',1.5);
    hold on;
    figure(4);
    h1b = fill(yfill,xfill,mapP(3,:),'EdgeColor','none');
    set(h1b,'FaceAlpha',alpha);
    p1b = plot(detTarget_mi_interp,dataNodeMean_mi,'-','Color',mapP(4,:),'linewidth',1.5);
    
    figure(3);
    xlabel('Nodes Detected (%)')
    ylabel('Targets Detected (%)')
    set(gca,'FontName','Arial','FontSize',fs)
    grid on;
    grid minor;
    axis square;
    set(gcf,'units','inches','position',[5,5,4,3])
    %axis equal;
    set(gcf,'Color','w')
    xlim([0 100]);
    ylim([0 100]);
    %xticks([0 0.2 0.4 0.6 0.8 1]);
    if ( j == 1)
        hlg = legend([p3 p1 p2], {'Mutual Info','Random','Lawnmower'},'Location','NW');
    end
    hlg.FontSize=fs;
    
    figure(4);
    ylabel('Nodes Detected (%)')
    xlabel('Targets Detected (%)')
    set(gca,'FontName','Arial','FontSize',fs)
    grid on;
    box on;
    grid minor;
    axis square;
    set(gcf,'units','inches','position',[5,5,4,3])
    set(gcf,'Color','w')
    xlim([0 100]);
    ylim([0 100]);
    xticks([0 0.2 0.4 0.6 0.8 1]*100);
    %title(labels{j})
    if ( j == 1)
        %hlg = legend([p3 p1 p2], {'Mutual Info','Random','Lawnmower'},'Location','NW');
    end
    hlg.FontSize=fs;
    
    
    
    %% Old:
    %     % generate plots of mean/stdev of discoveredNodePercentage
    %     figure;
    %     alpha = 0.25;
    %     timeRange = [1:1:maxPts];
    %     [plotHandle1] = plot(timeRange, meanOmitNaN(alg{1}.discoveredNodePercentageMat,1), 'k','linewidth',2);
    %     hold on;
    %     [plotHandle2] = plotMonteCarlo(timeRange, alg{2}.discoveredNodePercentageMat, 'r', alpha);
    %     [plotHandle3] = plotMonteCarlo(timeRange, alg{3}.discoveredNodePercentageMat, 'b', alpha);
    %     xlabel('Time (sec)')
    %     ylabel('Percent Nodes Discovered')
    %     set(gca,'FontSize',16)
    %     axis tight;
    %     ylim([0 100]);
    %     legend([plotHandle1 plotHandle2 plotHandle3],{'Mutual Info','Lawnmower','Random Wpts'})
    %
    %     % generate plots of mean/stdev of discoveredNodePercentage
    %     figure;
    %     alpha = 0.25;
    %     timeRange = [1:1:maxPts];
    %     [plotHandle1] = plot(timeRange, meanOmitNaN(alg{1}.totalEntropyMat,1), 'k','linewidth',2);
    %     hold on;
    %     [plotHandle2] = plotMonteCarlo(timeRange, alg{2}.totalEntropyMat, 'r', alpha);
    %     [plotHandle3] = plotMonteCarlo(timeRange, alg{3}.totalEntropyMat, 'b', alpha);
    %     xlabel('Time (sec)')
    %     ylabel('Entropy (bits)')
    %     set(gca,'FontSize',16)
    %     axis tight;
    %     %ylim([0 100]);
    %     legend([plotHandle1 plotHandle2 plotHandle3],{'Mutual Info','Lawnmower','Random Wpts'})
    
end

function [meanHandle] = plotMonteCarlo(x, data, c, alpha)
dataMean = mean(data,1);
numRuns = size(data,1);
numDataPts = size(data,2);
dataStdev = std(data,1);
meanHandle = plot(x,dataMean,[c '-'],'linewidth',2);
hold on;
h = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],c,'EdgeColor','none');
set(h,'FaceAlpha',alpha);
end