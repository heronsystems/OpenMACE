clear;
close all;
clc;
addpath('./external/brewer')
addpath('./external/export_fig')

alg_lm_range = 13; %[1 3 5 11 13 15 21 23 25];
alg_mi_range = 38; %[26 28 30 36 38 40 46 48 50];
alg_rm_range = 63; %[51 53 55 61 63 65 71 73 75];

processFlag = 0;
plotAll = 0;
%% data process
if ( processFlag )
    numTrials = 100;
    
    % mutual info
    disp('processing mutual info');
    load('plot_minfo_100.mat','alg');
    numCellsTotal = size(alg{38}.trial{1}.cellStateHist{1},1)*size(alg{38}.trial{1}.cellStateHist{1},2);
    maxPts = length(alg{38}.trial{1}.t);
    algRange = alg_mi_range;    % create matrix of data for each alg (trials x time)
    m = 1;
    for i = algRange
        alg_minfo{m}.discoveredNodePercentageMat = NaN*ones(numTrials,maxPts);
        for j = 1:1:numTrials
            numPts = length(alg{i}.trial{j}.t);
            for k = 1:1:numPts % total target motions
                alg_minfo{m}.discoveredNodePercentageMat(j,k) = alg{i}.trial{j}.discoveredNodePercentage(k);
            end
        end
        m  = m + 1;
    end
    clear alg;
    
    % lawnmower
    disp('processing lawnmower info');
    load('plot_lawnmower_100.mat','alg');
    algRange = alg_lm_range;    % create matrix of data for each alg (trials x time)
    m = 1;
    for i = algRange
        alg_lm{m}.discoveredNodePercentageMat = NaN*ones(numTrials,maxPts);
        for j = 1:1:numTrials
            numPts = length(alg{i}.trial{j}.t);
            for k = 1:1:numPts % total target motions
                alg_lm{m}.discoveredNodePercentageMat(j,k) = alg{i}.trial{j}.discoveredNodePercentage(k);
            end
        end
        m = m + 1;
    end
    clear alg;
    
    
    % random
    disp('processing random info');
    load('plot_random_100.mat','alg');
    algRange = alg_rm_range;    % create matrix of data for each alg (trials x time)
    m = 1;
    for i = algRange
        alg_rm{m}.discoveredNodePercentageMat = NaN*ones(numTrials,maxPts);
        for j = 1:1:numTrials
            numPts = length(alg{i}.trial{j}.t);
            for k = 1:1:numPts % total target motions
                alg_rm{m}.discoveredNodePercentageMat(j,k) = alg{i}.trial{j}.discoveredNodePercentage(k);
            end
        end
        m = m + 1;
    end
    clear alg;
    
else
    load('plot_curves_nodes_100.mat');
    if ( plotAll )
        labels={'(Poor,Poor)','(Poor,Fair)','(Poor,Excellent)','(Fair,Poor)','(Fair,Fair)','(Fair,Excellent)','(Excellent,Poor)','(Excellent,Fair)','(Excellent,Excellent)','Random','Lawnmower','Mutual Info.'};
        for i=1:1:9
            timeRange = [];
            
            % set lawnmower colors
            mapB = brewermap(4,'Blues');
            mapG = brewermap(4,'Greens');
            mapP = brewermap(4,'Purples');
            font = 'Arial';
            %figh = figure('DefaultTextFontName', font, 'DefaultAxesFontName', font);
            alpha = 0.5;
            timeRange = [1:1:maxPts];
            hold on;
            x = timeRange/(4*60);
            
            % plot random motion
            data = alg_rm{i}.discoveredNodePercentageMat;
            dataMean = meanOmitNaN(data,1);
            dataStdev = stdevOmitNaN(data,1);
            figure(1);
            subplot(3,3,i)
            p1 = plot(x,dataMean,'Color',mapB(4,:),'linewidth',1);
            hold on;
            h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapB(3,:),'EdgeColor','none');
            set(h1,'FaceAlpha',alpha);
            axis tight;
            
            % plot lawnmower
            subplot(3,3,i)
            data = alg_lm{i}.discoveredNodePercentageMat;
            dataMean = meanOmitNaN(data,1);
            dataStdev = stdevOmitNaN(data,1);
            p2 = plot(x,dataMean,'Color',mapG(4,:),'linewidth',1);
            hold on;
            h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapG(3,:),'EdgeColor','none');
            set(h1,'FaceAlpha',alpha);
            axis tight;
            
            % plot mutual Info
            subplot(3,3,i)
            data = alg_minfo{i}.discoveredNodePercentageMat;
            dataMean = meanOmitNaN(data,1);
            dataStdev = stdevOmitNaN(data,1);
            p3 = plot(x,dataMean,'Color',mapP(4,:),'linewidth',1);
            hold on;
            h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapP(3,:),'EdgeColor','none');
            set(h1,'FaceAlpha',alpha);
            axis tight;
            %set(gca,'FontName','Arial','FontSize',10)
            %grid on;
            %grid minor;
            %legend([p1 p2 p3],{'Random','Lawnmower','Mutual Info'}); %,'Location','SW');
            ylim([0 100])
            xlim([0 1])
            xlabel('Normalized Time')
            ylabel('Nodes Detected (%)')
            set(gca,'FontName','Arial','FontSize',12)
            title(labels{i})
            set(gcf,'Color','w')
            box on;
            set(gcf,'units','centimeters','position',[5,5,12,9])
            set(gcf,'units','centimeters','position',[5,5,12,9])
            box on;
        end
    else
        timeRange = [];
        fs = 10;
        % run
        %i = 3; % poor
        %i = 6; % Fair
        %i = 9 ; % excellent
        %i = 5 % (fair, fair)
        i = 1; % 100 trials
        
        % set lawnmower colors
        mapB = brewermap(4,'Blues');
        mapG = brewermap(4,'Greens');
        mapP = brewermap(4,'Purples');
        font = 'Arial';
        %figh = figure('DefaultTextFontName', font, 'DefaultAxesFontName', font);
        alpha = 0.5;
        timeRange = [1:1:maxPts];
        hold on;
        x = timeRange./(4*60);
        
        % plot random motion
        data = alg_rm{i}.discoveredNodePercentageMat;
        dataMean = meanOmitNaN(data,1);
        dataStdev = stdevOmitNaN(data,1);
        fig=figure(1);
        hold on;
        h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapB(3,:),'EdgeColor','none');
        p1 = plot(x,dataMean,'Color',mapB(4,:),'linewidth',1.5);        
        set(h1,'FaceAlpha',alpha);
        axis tight;
        
        % plot lawnmower
        data = alg_lm{i}.discoveredNodePercentageMat;
        dataMean = meanOmitNaN(data,1);
        dataStdev = stdevOmitNaN(data,1);
        hold on;
        h1 = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],mapG(3,:),'EdgeColor','none');
        p2 = plot(x,dataMean,'Color',mapG(4,:),'linewidth',1.5);        
        set(h1,'FaceAlpha',alpha);
        axis tight;
        
        % plot mutual Info
        data = alg_minfo{i}.discoveredNodePercentageMat;
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
        %         set(gcf,'units','centimeters','position',[5,5,12,9])
        %         box on;
        %print(1,'-dpdf','mc_gridPoor.pdf');
        %print(1,'-dpdf','mc_nodes.pdf');
        %print(1,'-dpdf','mc_gridExcellent.pdf');
    
    end
end