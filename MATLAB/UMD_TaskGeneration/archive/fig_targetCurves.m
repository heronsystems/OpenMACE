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

% (target, grid) = (Fair,Fair)
alg_lm_range = 13; % (mz,mg) = (2.5,2.5)
alg_mi_range = 38; 
alg_rm_range = 63;

% (target, grid) = (Fair,excellent)
%alg_lm_range = 25; % (mz,mg) = (2.5,3.5)
%alg_mi_range = 50; 
%alg_rm_range = 75;

%alg_lm_range = [1 3 5 11 13 15 21 23 25];
%alg_mi_range = [26 28 30 36 38 40 46 48 50];
%alg_rm_range = [51 53 55 61 63 65 71 73 75];


labels={'(Poor,Poor)','(Poor,Fair)','(Poor,Excellent)','(Fair,Poor)','(Fair,Fair)','(Fair,Excellent)','(Excellent,Poor)','(Excellent,Fair)','(Excellent,Excellent)','Random','Lawnmower','Mutual Info.'};



for j = 1:1:length(alg_lm_range)
    % % (excellent,excellent)
    % alg_lm = 25; %13;
    % alg_mi = 50; %38;
    % alg_rm = 75; %63;
    
    alg_lm = alg_lm_range(j);
    alg_rm = alg_rm_range(j);
    alg_mi = alg_mi_range(j);
    
    
    detTime_rm = [];
    detTime_lm = [];
    detTime_mi = [];
    for i = 1:1:100
        if ( detectionValidMat_rm(alg_rm,i) == 1 )
            detTime_rm =  [detTime_rm detectionTimeMat_rm(alg_rm,i)];
        end
        if ( detectionValidMat_mi(alg_mi,i) == 1 )
            detTime_mi =  [detTime_mi detectionTimeMat_mi(alg_mi,i)];
        end
        if ( detectionValidMat_lm(alg_lm,i) == 1 )
            detTime_lm =  [detTime_lm detectionTimeMat_lm(alg_lm,i)];
        end
    end
    
    
    % sort
    detTime_rm = sort(detTime_rm);
    detTime_lm = sort(detTime_lm);
    detTime_mi = sort(detTime_mi);
    
    % plot
    figure(1);
    %subplot(3,3,j);
    p1 = plot(detTime_rm/(4*60),[1:1:length(detTime_rm)]./100*100,'-','Color',mapB(4,:),'linewidth',1.5);
    hold on;
    p2 = plot(detTime_lm/(4*60),[1:1:length(detTime_lm)]./100*100,'-','Color',mapG(4,:),'linewidth',1.5);
    p3 = plot(detTime_mi/(4*60),[1:1:length(detTime_mi)]./100*100,'-','Color',mapP(4,:),'linewidth',1.5);
    xlabel('Normalized Time')
    ylabel('Targets Detected (%)')
    set(gca,'FontName','Arial','FontSize',10)
    grid on;
    grid minor;
    set(gcf,'Color','w')
    xlim([0 1]);
    ylim([0 100]);
    xticks([0 0.2 0.4 0.6 0.8 1]);
    %title(labels{j})
    if ( j == 1 )    
    %hlg = legend([p3 p1 p2], {'Mutual Info','Random','Lawnmower'},'Location','NW');
    end
    hlg.FontSize=10;
end
axis square;
set(gcf,'units','inches','position',[5,5,4,3])

%print(1,'-dpdf','mc_target.pdf');
%print(1,'-dpdf','mc_targetFair.pdf');
%print(1,'-dpdf','mc_targetExcellent.pdf');

%export_fig 'mc_targetPoor.pdf'
%export_fig 'mc_targetFair.pdf'
%export_fig 'mc_targetExcellent.pdf'







