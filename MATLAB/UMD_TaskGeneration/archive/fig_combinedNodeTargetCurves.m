clear;
close all;
clc;
addpath('./external/brewer')
addpath('./external/export_fig')

load('plot_random_100.mat','detectionTimeMat','detectionValidMat');
detectionTimeMat_rm  = detectionTimeMat;
detectionValidMat_rm = detectionValidMat;
clear('detectionTimeMat','detectionValidMat');

load('plot_lawnmower_100.mat','meanDetectionTimes','minDetectionTimes','maxDetectionTimes','stdDetectionTimes','detectionTimeMat','detectionValidMat');
detectionTimeMat_lm  = detectionTimeMat;
detectionValidMat_lm = detectionValidMat;
clear('detectionTimeMat','detectionValidMat');

load('plot_minfo_100.mat','meanDetectionTimes','minDetectionTimes','maxDetectionTimes','stdDetectionTimes','detectionTimeMat','detectionValidMat');
detectionTimeMat_mi  = detectionTimeMat;
detectionValidMat_mi = detectionValidMat;
clear('detectionTimeMat','detectionValidMat');

% line colors
mapP = brewermap(4,'Purples');
mapG = brewermap(4,'Greens');
mapB = brewermap(4,'Blues');

% first test, lawnmower
% [1 3 5];
%xx [6,8,10];
% [11 13 15];
%xx [16,18,20];
% [21 23 25]

% first test mutual info
% [26,28,30]; % each of these rows is a fixed mG
%xx [31,33,35];
% [36,38,40];
%xx [41,43,45];
% [46,48,50]

% first test, random
% [51,53,55];
%xx [56,58,60];
% [61,63,65];
%xx [41,43,45];
% [71,73,75]

% %(target, grid) = (poor,excellent)
% alg_lm_range = 21; % (mz,mg) = (1.5,3.5)
% alg_mi_range = 46;
% alg_rm_range = 71;
%
% % (target, grid) = (Fair,excellent)
% alg_lm_range = 23; % (mz,mg) = (2.5,3.5)
% alg_mi_range = 48;
% alg_rm_range = 73;

% (target, grid) = (Fair,excellent)
%alg_lm_range = 25; % (mz,mg) = (2.5,3.5)
%alg_mi_range = 50;
%alg_rm_range = 75;



% alg_lm_range = [1 3 5 11 13 15 21 23 25];
% alg_mi_range = [26 28 30 36 38 40 46 48 50];
% alg_rm_range = [51 53 55 61 63 65 71 73 75];


% %
%         case 1 = (grid, target) = (poor, poor)
%             swarmModel.mZ = 1.5; % target sensor [poor]
%             swarmModel.mG = 1.5; % grid sensor [poor]
%         case 3 = (grid, target) = (poor, fair)
%             swarmModel.mZ = 2.5; % target sensor [fair]
%             swarmModel.mG = 1.5; % grid sensor [poor]
%         case 5 = (grid, target) = (poor, excellent)
%             swarmModel.mZ = 3.5; % target sensor [excellent]
%             swarmModel.mG = 1.5; % grid sensor [poor]

labels={'(Poor,Poor)','(Poor,Fair)','(Poor,Excellent)','(Fair,Poor)','(Fair,Fair)','(Fair,Excellent)','(Excellent,Poor)','(Excellent,Fair)','(Excellent,Excellent)','Random','Lawnmower','Mutual Info.'};

load('plot_curves_nodes.mat');

% (target, grid) = (Fair,Fair)
alg_lm_range = 13; % (mz,mg) = (2.5,2.5)
alg_mi_range = 38;
alg_rm_range = 63;

% alg_lm_range = [1 3 5 11 13 15 21 23 25];
%
alg_lm_index(1) = 1;
alg_lm_index(3) = 2;
alg_lm_index(5) = 3;

alg_lm_index(11) = 4;
alg_lm_index(13) = 5;
alg_lm_index(15) = 6;

alg_lm_index(21) = 7;
alg_lm_index(23) = 8;
alg_lm_index(25) = 9;

% alg_mi_range = [26 28 30 36 38 40 46 48 50];
%
alg_mi_index(26) = 1;
alg_mi_index(28) = 2;
alg_mi_index(30) = 3;

alg_mi_index(36) = 4;
alg_mi_index(38) = 5;
alg_mi_index(40) = 6;

alg_mi_index(46) = 7;
alg_mi_index(48) = 8;
alg_mi_index(50) = 9;

% alg_rm_range = [51 53 55 61 63 65 71 73 75];
%
alg_rm_index(51) = 1;
alg_rm_index(53) = 2;
alg_rm_index(55) = 3;

alg_rm_index(61) = 4;
alg_rm_index(63) = 5;
alg_rm_index(65) = 6;

alg_rm_index(71) = 7;
alg_rm_index(73) = 8;
alg_rm_index(75) = 9;


fs = 10;
for j = 1:1:length(alg_lm_range)
    % % (excellent,excellent)
    % alg_lm = 25; %13;
    % alg_mi = 50; %38;
    % alg_rm = 75; %63;
    
    alg_lm_ind = alg_lm_range(j);
    alg_rm_ind = alg_rm_range(j);
    alg_mi_ind = alg_mi_range(j);
    
    
    font = 'Arial';
    alpha = 0.5;
    timeRange = [1:1:maxPts];
    hold on;
    x = timeRange;
    
    
    detTime_rm = [];
    detTime_lm = [];
    detTime_mi = [];
    for i = 1:1:100
        if ( detectionValidMat_rm(alg_rm_ind,i) == 1 )
            detTime_rm =  [detTime_rm detectionTimeMat_rm(alg_rm_ind,i)];
        end
        if ( detectionValidMat_mi(alg_mi_ind,i) == 1 )
            detTime_mi =  [detTime_mi detectionTimeMat_mi(alg_mi_ind,i)];
        end
        if ( detectionValidMat_lm(alg_lm_ind,i) == 1 )
            detTime_lm =  [detTime_lm detectionTimeMat_lm(alg_lm_ind,i)];
        end
    end
    
    
    % sort
    detTime_rm = sort(detTime_rm);
    detTime_lm = sort(detTime_lm);
    detTime_mi = sort(detTime_mi);
    
    %%
    % get random motion data
    data = alg_rm{ alg_rm_index(alg_rm_ind) }.discoveredNodePercentageMat;
    dataNodeMean_rm = meanOmitNaN(data,1);
    dataNodeStdev_rm = stdevOmitNaN(data,1);
    clear data;
    % get lm data
    data = alg_lm{ alg_lm_index(alg_lm_ind) }.discoveredNodePercentageMat;
    dataNodeMean_lm = meanOmitNaN(data,1);
    dataNodeStdev_lm = stdevOmitNaN(data,1);
    % get
    data = alg_minfo{ alg_mi_index(alg_mi_ind) }.discoveredNodePercentageMat;
    dataNodeMean_mi = meanOmitNaN(data,1);
    dataNodeStdev_mi = stdevOmitNaN(data,1);
    
    
    % augment detection times
    detTime_rm = [ 0 detTime_rm ];
    detTime_lm = [ 0 detTime_lm ];
    detTime_mi = [ 0 detTime_mi ];
    
    detPercent_rm = [0:1:length(detTime_rm)-1]./100;
    detPercent_lm = [0:1:length(detTime_lm)-1]./100;
    detPercent_mi = [0:1:length(detTime_mi)-1]./100;
    
    % make unique
    [detTime_rm, ind] = unique(detTime_rm,'last');
    detPercent_rm = detPercent_rm(ind);
    
    [detTime_lm, ind] = unique(detTime_lm,'last');
    detPercent_lm = detPercent_lm(ind);
    
    [detTime_mi, ind] = unique(detTime_mi,'last');
    detPercent_mi = detPercent_mi(ind);
    
    
    % need to interpolate to same number of points
    timeRange = [1:1:length(alg_rm{1}.discoveredNodePercentageMat)];
    method = 'nearest';
    detTarget_rm_interp = interp1(detTime_rm, detPercent_rm, timeRange,method)*100;
    detTarget_lm_interp = interp1(detTime_lm, detPercent_lm, timeRange,method)*100;
    detTarget_mi_interp = interp1(detTime_mi, detPercent_mi, timeRange,method)*100;
    
    
    alpha = 0.5;
    
    
    % plot
    figure(1);
    %subplot(3,3,j);
    hold on;
    %h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapG(3,:),'EdgeColor','none');
    indNan = isnan(detTarget_rm_interp);
    detTarget_rm_interp(indNan) = [];
    dataNodeMean_rm(indNan) = [];
    dataNodeStdev_rm(indNan) = [];
    %timeRange_rm(indNan) = [];
    
    [detTarget_rm_interp_fill_first,ind_first] = unique(detTarget_rm_interp,'first');
    [detTarget_rm_interp_fill_last,ind_last] = unique(detTarget_rm_interp,'last');
    dataNodeMean_rm_fill_first = dataNodeMean_rm(ind_first);
    dataNodeStdev_rm_fill_first = dataNodeStdev_rm(ind_first);
    dataNodeMean_rm_fill_last = dataNodeMean_rm(ind_last);
    dataNodeStdev_rm_fill_last = dataNodeStdev_rm(ind_last);
    xfill = [dataNodeMean_rm_fill_first-dataNodeStdev_rm_fill_first fliplr(dataNodeMean_rm_fill_last+dataNodeStdev_rm_fill_last)];
    yfill = [detTarget_rm_interp_fill_first fliplr(detTarget_rm_interp_fill_last)];
    h1 = fill(xfill,yfill,mapB(3,:),'EdgeColor','none');
    set(h1,'FaceAlpha',alpha);
    p1 = plot(dataNodeMean_rm,detTarget_rm_interp,'-','Color',mapB(4,:),'linewidth',1.5);
    
    figure(2);
    %subplot(3,3,j);
    hold on;
    h1b = fill(yfill,xfill,mapB(3,:),'EdgeColor','none');
    set(h1b,'FaceAlpha',alpha);
    p1b = plot(detTarget_rm_interp,dataNodeMean_rm,'-','Color',mapB(4,:),'linewidth',1.5);
    
    figure(1);
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
    h2 = fill(xfill,yfill,mapG(3,:),'EdgeColor','none');
    set(h2,'FaceAlpha',alpha);
    p2 = plot(dataNodeMean_lm,detTarget_lm_interp,'-','Color',mapG(4,:),'linewidth',1.5);
    
    figure(2);
    %subplot(3,3,j);
    hold on;
    h1b = fill(yfill,xfill,mapG(3,:),'EdgeColor','none');
    set(h1b,'FaceAlpha',alpha);
    p1b = plot(detTarget_lm_interp,dataNodeMean_lm,'-','Color',mapG(4,:),'linewidth',1.5);
    
    figure(1);
    
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
    h3 = fill(xfill,yfill,mapP(3,:),'EdgeColor','none');
    set(h3,'FaceAlpha',alpha);
    p3 = plot(dataNodeMean_mi,detTarget_mi_interp,'-','Color',mapP(4,:),'linewidth',1.5);
    
    figure(2);
    %subplot(3,3,j);
    hold on;
    h1b = fill(yfill,xfill,mapP(3,:),'EdgeColor','none');
    set(h1b,'FaceAlpha',alpha);
    p1b = plot(detTarget_mi_interp,dataNodeMean_mi,'-','Color',mapP(4,:),'linewidth',1.5);
    
    figure(1);
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
    title(labels{j})
    if ( j == 1)
        hlg = legend([p3 p1 p2], {'Mutual Info','Random','Lawnmower'},'Location','NW');
    end
    hlg.FontSize=fs;
    
    figure(2);
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
    %     % plot
    %     figure(2);
    %     subplot(3,3,j);
    %     p1 = plot(timeRange/length(timeRange),(dataNodeMean_rm+detTarget_rm_interp)/2,'-','Color',mapB(4,:),'linewidth',1.5);
    %     hold on;
    %     p2 = plot(timeRange/length(timeRange),(dataNodeMean_lm+detTarget_lm_interp)/2,'-','Color',mapG(4,:),'linewidth',1.5);
    %     p3 = plot(timeRange/length(timeRange),(dataNodeMean_mi+detTarget_mi_interp)/2,'-','Color',mapP(4,:),'linewidth',1.5);
    %     xlabel('Normalized Time')
    %     ylabel('Nodes and Targets Detected (%)')
    %     set(gca,'FontName','Arial','FontSize',fs)
    % %     grid on;
    % %     grid minor;
    %     %axis equal;
    %     set(gcf,'Color','w')
    %     xlim([0 1]);
    %     ylim([0 100]);
    %     %axis square;
    %     %xticks([0 0.2 0.4 0.6 0.8 1]);
    %     title(labels{j})
    %     if ( j == 1)
    %     hlg = legend([p3 p1 p2], {'Mutual Info','Random','Lawnmower'},'Location','NW');
    %     end
    %     hlg.FontSize=fs;
    
    
    
end
%axis square;
%set(gcf,'units','inches','position',[5,5,4,3])

%print(2,'-dpdf','mc_combined.pdf');
%print(1,'-dpdf','mc_targetFair.pdf');
%print(1,'-dpdf','mc_targetExcellent.pdf');

%export_fig 'mc_targetPoor.pdf'
%export_fig 'mc_targetFair.pdf'
%export_fig 'mc_targetExcellent.pdf'







