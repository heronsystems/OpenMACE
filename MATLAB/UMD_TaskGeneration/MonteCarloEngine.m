%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A Monte Carlo Engine for main_taskGeneration.m
%
% Sheng Cheng, Oct 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear; close all; clc;
% format compact;
updatePath;

% turn on switch
MonteCarloSwitch = 1;
%rng('default');
%rng(1);

simulationOrPlot = 'sim'; %options are 'sim' or 'plot' or 'analysis', or 'analysisPlot'
% if strcmp(simulationOrPlot,'analysis')
%     numAlgs = 3;
%     numTrials = 4;
%     detectionFlag = zeros(numAlgs,numTrials);
%     detectionValid = zeros(numAlgs,numTrials);
%     detectionTime = zeros(numAlgs,numTrials);
% end


kk = 1;

        % first test, lawnmower
        % [1,3,5];
        %xx [6,8,10];
        % [11,13,15];
        %xx [16,18,20];
        % [21,23,25]
        
        % first test mutual info
        % [26,28,30];
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
        
    %algRange = 
    %trialRange = ;
    
for ii = [13,38,63] %%[26,28,30,36,38,40,46,48,50] ; %[26,28,30,36,38,40] % alg %  [1,3,5,11,13,15,21,23,25]
    for jj = [1 50 100]%[1:100]% total target motions
        %             ((kk-1)*1+jj)/(1*30)/3  % print percentage of completion
        if strcmp(simulationOrPlot,'sim')
            fprintf('Running algorithm %d (trial %d) \n', ii, jj);
            % set the ID of algorithm, initial formation, and target motion here (remember to load the IDs to the loadParams(ID1,ID2,ID3) function)
            algorithmID = ii;
            initialFormationID = jj; % default for corners (automatic)
            targetMotionID = jj;  %
            % run main program
            main_taskGeneration;
            clearvars -except simulationOrPlot ii kk jj MonteCarloSwitch;
            simulationOrPlot = 'sim';
        elseif strcmp(simulationOrPlot,'plot')
            fprintf('Plotting algorithm %d (trial %d) \n', ii, jj);
            clearvars -except ii kk jj simulationOrPlot algRange trialRange;
            close all;
            
            str = ['./monteCarloRuns/MonteCarlo_Algorithm' num2str(ii) '_InitialFormation' num2str(jj) '_TargetMotion' num2str(jj) '.mat'];
            load(str);
            plotPerformanceSnapshot(swarmWorldHist, swarmStateHist, targetStateHist, trueWorld, runParams, swarmModel, targetModel );
            
            %             figure(3);
            %             set(gcf,'units','normalized','outerposition',[0 0 1 1]);
            %             saveas(gcf,['Algo' num2str(ii) '_Formation' num2str(kk) '_Target' num2str(jj) '_Entropy' '.png']);
            %
%             figure(4);
%             set(gcf,'units','normalized','outerposition',[0 0 1 1]);
%             saveas(gcf,['Algo' num2str(ii) '_Formation' num2str(jj) '_Target' num2str(jj) '_Trajectory' '.png']);
%             
            %             figure(8)
            %             set(gcf,'units','normalized','outerposition',[0 0 1 1]);
            %             saveas(gcf,['Algo' num2str(ii) '_Formation' num2str(kk) '_Target' num2str(jj) '_PercentageDiscoverNodes' '.png']);
            %
            simulationOrPlot = 'plot';
            
        elseif strcmp(simulationOrPlot,'analysis')
            fprintf('Analyzing algorithm %d (trial %d) \n', ii, jj);
            str = ['./monteCarloRuns/MonteCarlo_Algorithm' num2str(ii) '_InitialFormation' num2str(jj) '_TargetMotion' num2str(jj) '.mat'];
            load(str,'swarmWorldHist','trueWorld');
            %disp('finished loading')
            % store relevant detection data
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
                %nodeDensityEstimate(i) = swarmWorldHist{i}.nodeDensityEstimate;
                % compute entropy of entire search grid
                entropyHist{i} = swarmWorldHist{i}.entropyMat;
                totalEntropy(i) = swarmWorldHist{i}.totalEntropy;
                
                %
                cellMsmtHist{i} = swarmWorldHist{i}.cellMsmtMat;
                cellStateHist{i} = swarmWorldHist{i}.cellStateMat;
                cellDetHist{i} = swarmWorldHist{i}.cellDetMat;
                numViews(i) = swarmWorldHist{i}.numViews;
                % compute the percentage of discovered nodes
                discoveredNodePercentage(i) = 100*numnodes(swarmWorldHist{i}.exploredGraph)/numnodes(trueWorld.G_env);
            end
            mapPercentage = swarmWorldHist{end}.mapPercentage;
            
            % pack into alg/trial structure so it is all in one place
            alg{ii}.trial{jj}.detectionFlag = detectionFlag;
            alg{ii}.trial{jj}.detectionValid = detectionValid;
            alg{ii}.trial{jj}.detectionTime = detectionTime;
            
            alg{ii}.trial{jj}.cellDetHist = cellDetHist;
            alg{ii}.trial{jj}.cellMsmtHist = cellMsmtHist;
            alg{ii}.trial{jj}.cellStateHist = cellStateHist;
            alg{ii}.trial{jj}.numViews = numViews;
            alg{ii}.trial{jj}.entropyHist = entropyHist;
            
            alg{ii}.trial{jj}.t = t;
            alg{ii}.trial{jj}.totalEntropy = totalEntropy;
            alg{ii}.trial{jj}.discoveredNodePercentage = discoveredNodePercentage;
            
            %
            alg{ii}.trial{jj}
            if isempty( alg{ii}.trial{1} )
                disp('stop');
            end
            alg{ii}.trial{1}
            simulationOrPlot = 'analysis';
            clearvars -except simulationOrPlot ii kk jj MonteCarloSwitch detection detectionValid detectionTime alg;
        end
    end
    %end
end


if (strcmp(simulationOrPlot,'analysisPlot'))
    
    
    load('./monteCarloRuns/MonteCarlo_Algorithm13_InitialFormation50_TargetMotion50.mat','trueWorld')
        % first test, lawnmower
        % algRange1;
        %xx [6,8,10];
        % algRange2;
        %xx [16,18,20];
        % algRange3
        
        % first test mutual info
        % [26,28,30];
        %xx [31,33,35];
        % [36,38,40];
        %xx [41,43,45];
        % [46,48,50]
        
    %algRange = [51,53,55,61,63,65,71,73,75]; %,; %,26,28,30,36,38,40,46,48,50];
    algRange = [63]%[1,3,5,11,13,15,21,23,25];
    trialRange = 1:100; %:25;
    
    mapPercentThresh = 0.10; %
    % scalar data is transformed into cell vector ( numAlgs x numTrials )
    for i = algRange
        numValidAlg = 0;
        numDetAlg = 0;
        %for kk = 1 % total initial formations (initial formation 1 is from corners)
        for j = trialRange % total target motions
            fprintf('Processing alg %d trial %d \n', i, j);
            % entropy
            entropyMat(i,j) = alg{i}.trial{j}.totalEntropy(end);
            entropyMidwayMat(i,j) = alg{i}.trial{j}.totalEntropy(floor(end/2));
            % det
            detectionFlagMat(i,j) = alg{i}.trial{j}.detectionFlag;
            detectionValidMat(i,j) = alg{i}.trial{j}.detectionValid;
            if ( detectionFlagMat(i,j)==1 && detectionValidMat(i,j) == 1 )
                detectionTimeMat(i,j) = alg{i}.trial{j}.detectionTime;
                numValidAlg = numValidAlg + 1;
            else
                detectionTimeMat(i,j) = NaN;
            end
            if ( detectionFlagMat(i,j)==1 == 1 )
                numDetAlg = numDetAlg + 1;
            end
            % time
            mapThreshTime = -1;
            for k = 1:1:length(alg{i}.trial{j}.cellDetHist)
                numNodesTotal = 0;
                numNodesValid = 0;
                for l = 1:1:trueWorld.numBinsX
                    for m = 1:1:trueWorld.numBinsY
                        % calculate number of valid nodes and total nodes
                        % detected
                        if( alg{i}.trial{j}.cellDetHist{k}(l,m) == 1 )
                            numNodesTotal = numNodesTotal + 1;
                            if ( trueWorld.numNodesMat(l,m) == 1)
                                numNodesValid = numNodesValid + 1;
                            end
                            if ( (mapThreshTime == -1) && (numNodesValid >= mapPercentThresh*trueWorld.numNodes) )
                                mapThreshTime = alg{i}.trial{j}.t(k) ;
                            end
                        end
                        % calculate the true entropy
                    end
                end
                % record entropy
            end        
            if (mapThreshTime == -1)
               error('mapThreshTime not found'); 
            end
            percentNodesTotalMat(i,j) = numNodesTotal/trueWorld.numNodes;
            percentNodesValidMat(i,j) = numNodesValid/trueWorld.numNodes;
            mapThreshTimeMat(i,j) = mapThreshTime;
        end
        numValid(i) = numValidAlg;
        numDet(i) = numDetAlg;
    end
    
    % plot histogram
    for i = 1:1:length(algRange)
        meanDetectionTimes(i) = meanOmitNaN(detectionTimeMat(algRange(i),:),2);
        stdDetectionTimes(i) = stdevOmitNaN(detectionTimeMat(algRange(i),:),2);
        minDetectionTimes(i) = min(detectionTimeMat(algRange(i),:));
        maxDetectionTimes(i) = max(detectionTimeMat(algRange(i),:));
        
        meanPercentNodesTotal(i) = meanOmitNaN(percentNodesTotalMat(algRange(i),:),2);
        stdPercentNodesTotal(i) = stdevOmitNaN(percentNodesTotalMat(algRange(i),:),2);
        minPercentNodesTotal(i) = min(percentNodesTotalMat(algRange(i),:));
        maxPercentNodesTotal(i) = max(percentNodesTotalMat(algRange(i),:));
        
        meanPercentNodesValid(i) = meanOmitNaN(percentNodesValidMat(algRange(i),:),2);
        stdPercentNodesValid(i) = stdevOmitNaN(percentNodesValidMat(algRange(i),:),2);
        minPercentNodesValid(i) = min(percentNodesValidMat(algRange(i),:));
        maxPercentNodesValid(i) = max(percentNodesValidMat(algRange(i),:));
        
        meanMapThreshTime(i) = meanOmitNaN(mapThreshTimeMat(algRange(i),:),2);
        stdMapThreshTime(i) = stdevOmitNaN(mapThreshTimeMat(algRange(i),:),2);
        minMapThreshTime(i) = min(mapThreshTimeMat(algRange(i),:));
        maxMapThreshTime(i) = max(mapThreshTimeMat(algRange(i),:));
        
        meanEntropy(i) = meanOmitNaN(entropyMat(algRange(i),:),2);
        stdEntropy(i) = stdevOmitNaN(entropyMat(algRange(i),:),2);     
        minEntropy(i) = min(entropyMat(algRange(i),:));
        maxEntropy(i) = max(entropyMat(algRange(i),:));
        
        meanEntropyMid(i) = meanOmitNaN(entropyMidwayMat(algRange(i),:),2);
        stdEntropyMid(i) = stdevOmitNaN(entropyMidwayMat(algRange(i),:),2);             
        minEntropyMid(i) = min(entropyMidwayMat(algRange(i),:));
        maxEntropyMid(i) = max(entropyMidwayMat(algRange(i),:));
    end
    meanDetectionTimes
    
        % first test, lawnmower
        % algRange1;
        %xx [6,8,10];
        % algRange2;
        %xx [16,18,20];
        % algRange3
        
        % first test mutual info
        % [26,28,30];
        %xx [31,33,35];
        % [36,38,40];
        %xx [41,43,45];
        % [46,48,50]
    
%         algRange1 = 1:3; %[1,3,5];
%         algRange2 = 4:6; %[11,13,15];
%         algRange3 = 7:9; %[21,23,25];
%         algNums1 = [1,3,5];
%         algNums2 = [11,13,15];
%         algNums3 = [21,23,25];
%         algName = 'Lawnmower';
%         
%         algRange1 = 1:3; %[1,3,5];
%         algRange2 = 4:6; %[11,13,15];
%         algRange3 = 7:9; %[21,23,25];
%         algNums1 = [26,28,30];
%         algNums2 = [36,38,40];
%         algNums3 = [46,48,50];
%         algName = 'MutualInfo';
%         
%     figure;
%     errorbar([1.5 2.5 3.5],meanPercentNodesValid(algRange1),stdPercentNodesValid(algRange1),'o-','MarkerSize',16,'linewidth',2);hold on;
%     errorbar([1.5 2.5 3.5],meanPercentNodesValid(algRange2),stdPercentNodesValid(algRange2),'o-','MarkerSize',16,'linewidth',2);
%     errorbar([1.5 2.5 3.5],meanPercentNodesValid(algRange3),stdPercentNodesValid(algRange3),'o-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     legend('mG = 1.5', 'mG = 2.5' , 'mG = 3.5');
%     ylabel('Percent of Nodes Detected')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0.5 1])
%     title(algName);
%     
%     figure;
%     errorbar([1.5 2.5 3.5],meanDetectionTimes(algRange1),stdDetectionTimes(algRange1),'o-','MarkerSize',16,'linewidth',2);hold on;
%     errorbar([1.5 2.5 3.5],meanDetectionTimes(algRange2),stdDetectionTimes(algRange2),'o-','MarkerSize',16,'linewidth',2);
%     errorbar([1.5 2.5 3.5],meanDetectionTimes(algRange3),stdDetectionTimes(algRange3),'o-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     legend('mG = 1.5', 'mG = 2.5' , 'mG = 3.5');
%     ylabel('Target Detection Time (s)')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0.5 1])
%     title(algName);
%     
%     
%     figure;
%     errorbar([1.5 2.5 3.5],meanMapThreshTime(algRange1),stdMapThreshTime(algRange1),'o-','MarkerSize',16,'linewidth',2);hold on;
%     errorbar([1.5 2.5 3.5],meanMapThreshTime(algRange2),stdMapThreshTime(algRange2),'o-','MarkerSize',16,'linewidth',2);
%     errorbar([1.5 2.5 3.5],meanMapThreshTime(algRange3),stdMapThreshTime(algRange3),'o-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     legend('mG = 1.5', 'mG = 2.5' , 'mG = 3.5');
%     ylabel('Time to Map 70% of Nodes (s)')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0.5 1])
%     title(algName);
%     
%     figure;
%     errorbar([1.5 2.5 3.5],meanEntropy(algRange1),stdEntropy(algRange1),'o-','MarkerSize',16,'linewidth',2);hold on;
%     errorbar([1.5 2.5 3.5],meanEntropy(algRange2),stdEntropy(algRange2),'o-','MarkerSize',16,'linewidth',2);
%     errorbar([1.5 2.5 3.5],meanEntropy(algRange3),stdEntropy(algRange3),'o-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     legend('mG = 1.5', 'mG = 2.5' , 'mG = 3.5');
%     ylabel('Entropy after 6 min. (bits)')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0.5 1])
%     title(algName);    
% 
%     figure;
%     errorbar([1.5 2.5 3.5],meanEntropyMid(algRange1),stdEntropyMid(algRange1),'o-','MarkerSize',16,'linewidth',2);hold on;
%     errorbar([1.5 2.5 3.5],meanEntropyMid(algRange2),stdEntropyMid(algRange2),'o-','MarkerSize',16,'linewidth',2);
%     errorbar([1.5 2.5 3.5],meanEntropyMid(algRange3),stdEntropyMid(algRange3),'o-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     legend('mG = 1.5', 'mG = 2.5' , 'mG = 3.5');
%     ylabel('Entropy after 3 min. (bits)')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0.5 1])
%     title(algName); 
%     
%     
%     
%     figure;
%     plot([1.5 2.5 3.5],numValid(algNums1),'o-','MarkerSize',16,'linewidth',2);hold on;
%     plot([1.5 2.5 3.5],numValid(algNums2),'o-','MarkerSize',16,'linewidth',2);
%     plot([1.5 2.5 3.5],numValid(algNums3),'o-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     legend('mG = 1.5', 'mG = 2.5' , 'mG = 3.5');
%     ylabel('Valid Target Detections')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0.5 1])
%     title(algName);     
%     
%     figure;
%     plot([1.5 2.5 3.5],numDet(algNums1)-numValid(algNums1),'o-','MarkerSize',16,'linewidth',2); hold on;
%     plot([1.5 2.5 3.5],numDet(algNums2)-numValid(algNums2),'o-','MarkerSize',16,'linewidth',2);
%     plot([1.5 2.5 3.5],numDet(algNums3)-numValid(algNums3),'o-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     legend('mG = 1.5', 'mG = 2.5' , 'mG = 3.5');
%     ylabel('Target False Alarms')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0.5 1])
%     title(algName);     
%     
%     
% 
% 
%     
% %     
% %     % plot detection time
% %     
% %     algName = 'Random';
% %     %
% %     figure(1);
% %     errorbar([1 2 3],meanPercentNodesValid(1:3),stdPercentNodesValid(1:3),'o-','MarkerSize',16,'linewidth',2);
% %     hold on;
% %     errorbar([1 2 3],meanPercentNodesValid(4:6),stdPercentNodesValid(4:6),'x-','MarkerSize',16,'linewidth',2);
% %     errorbar([1 2 3],meanPercentNodesValid(7:9),stdPercentNodesValid(7:9),'s-','MarkerSize',16,'linewidth',2);
% %     xlabel('Target Sensor Sensitivity, mZ')
% %     ylabel('Percent of Valid Nodes Detected')
% %     grid on;    
% %     set(gca,'FontSize',16)
% %     ylim([0.5 1])
% %     legend('mG = 1','mG = 2','mG = 3')
% %     title(algName);
% %     
% %     %
% %     figure(2);
% %     errorbar([1 2 3],meanPercentNodesTotal(1:3),stdPercentNodesTotal(1:3),'o-','MarkerSize',16,'linewidth',2);
% %     hold on;
% %     errorbar([1 2 3],meanPercentNodesTotal(4:6),stdPercentNodesTotal(4:6),'x-','MarkerSize',16,'linewidth',2);
% %     errorbar([1 2 3],meanPercentNodesTotal(7:9),stdPercentNodesTotal(7:9),'s-','MarkerSize',16,'linewidth',2);
% %     xlabel('Target Sensor Sensitivity, mZ')
% %     ylabel('Ratio Detected/Actual Nodes')
% %     grid on;
% %     set(gca,'FontSize',16)
% %     ylim([0.75 1.25])
% %     legend('mG = 1','mG = 2','mG = 3')
% %     title(algName);
% %     
% %     
% %     %
% %     figure(3);
% %     errorbar([1 2 3],meanMapThreshTime(1:3),stdMapThreshTime(1:3),'o-','MarkerSize',16,'linewidth',2);
% %     hold on;
% %     errorbar([1 2 3],meanMapThreshTime(4:6),stdMapThreshTime(4:6),'x-','MarkerSize',16,'linewidth',2);
% %     errorbar([1 2 3],meanMapThreshTime(7:9),stdMapThreshTime(7:9),'s-','MarkerSize',16,'linewidth',2);
% %     xlabel('Target Sensor Sensitivity, mZ')
% %     ylabel('Time to Map 70% Valid Nodes (s)')
% %     grid on;    
%     set(gca,'FontSize',16)
%     legend('mG = 1','mG = 2','mG = 3')
%     ylim([0 360])
%     title(algName);
%     
%     %
%     figure(4);
%     errorbar([1 2 3],meanDetectionTimes(1:3),stdDetectionTimes(1:3),'o-','MarkerSize',16,'linewidth',2);
%     hold on;
%     errorbar([1 2 3],meanDetectionTimes(4:6),stdDetectionTimes(4:6),'x-','MarkerSize',16,'linewidth',2);
%     errorbar([1 2 3],meanDetectionTimes(7:9),stdDetectionTimes(7:9),'s-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     ylabel('Target Detection Time (s)')
%     grid on;    
%     set(gca,'FontSize',16)
%     ylim([0 360])
%     xlim([1 3])
%     legend('mG = 1','mG = 2','mG = 3')
%     title(algName);
% 
%     figure(5);
%     plot([1 2 3],numValid(algRange(1:3)),'o-','MarkerSize',16,'linewidth',2);
%     hold on;
%     plot([1 2 3],numValid(algRange(4:6)),'x-','MarkerSize',16,'linewidth',2);
%     plot([1 2 3],numValid(algRange(7:9)),'s-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     ylabel('Number Valid Detections')
%     grid on;    
%     set(gca,'FontSize',16)
%     legend('mG = 1','mG = 2','mG = 3')
%     ylim([0 15])
%     title(algName);
%     
%     figure(6);
%     plot([1 2 3],numDet(algRange(1:3)),'o-','MarkerSize',16,'linewidth',2);
%     hold on;
%     plot([1 2 3],numDet(algRange(4:6)),'x-','MarkerSize',16,'linewidth',2);
%     plot([1 2 3],numDet(algRange(7:9)),'s-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     ylabel('Number Total Detections')
%     grid on;    
%     set(gca,'FontSize',16)
%     legend('mG = 1','mG = 2','mG = 3')
%     ylim([0 15])
%     title(algName);
%     
%     
%     %
%     figure(7);
%     errorbar([1 2 3],meanEntropy(1:3),stdEntropy(1:3),'o-','MarkerSize',16,'linewidth',2);
%     hold on;
%     errorbar([1 2 3],meanEntropy(4:6),stdEntropy(4:6),'x-','MarkerSize',16,'linewidth',2);
%     errorbar([1 2 3],meanEntropy(7:9),stdEntropy(7:9),'s-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     ylabel('Final Entropy (bits)')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0 360])
%     xlim([1 3])
%     legend('mG = 1','mG = 2','mG = 3')
%     title(algName);  
% 
%     
%     %
%     figure(8);
%     errorbar([1 2 3],meanEntropyMid(1:3),stdEntropyMid(1:3),'o-','MarkerSize',16,'linewidth',2);
%     hold on;
%     errorbar([1 2 3],meanEntropyMid(4:6),stdEntropyMid(4:6),'x-','MarkerSize',16,'linewidth',2);
%     errorbar([1 2 3],meanEntropyMid(7:9),stdEntropyMid(7:9),'s-','MarkerSize',16,'linewidth',2);
%     xlabel('Target Sensor Sensitivity, mZ')
%     ylabel('Entropy At 3 min. (bits)')
%     grid on;    
%     set(gca,'FontSize',16)
%     %ylim([0 800])
%     xlim([1 3])
%     legend('mG = 1','mG = 2','mG = 3')
%     title(algName);  
% 
%     
%     print(1,'-dpdf',[algName '_nodePercentValid.pdf'])    
%     print(2,'-dpdf',[algName '_nodeRatioDetActual.pdf'])
%     print(3,'-dpdf',[algName '_nodeMapTimeThresh.pdf'])
%     print(4,'-dpdf',[algName '_targDetTime.pdf'])
%     print(5,'-dpdf',[algName '_targValidDetections.pdf'])
%     print(6,'-dpdf',[algName '_targTotalDetections.pdf'])
%     print(7,'-dpdf',[algName '_entropy.pdf'])  
%     print(8,'-dpdf',[algName '_entropy3.pdf'])      
%     
%     
    %
    %     detTimeMatrix(1,1) = meanDetectionTimes(1);
    %     detTimeMatrix(1,2) = meanDetectionTimes(1);
    %     detTimeMatrix(1,3) = meanDetectionTimes(1);
    %     detTimeMatrix(1,1) = meanDetectionTimes(1);
    %     detTimeMatrix(1,1) = meanDetectionTimes(1);
    %     detTimeMatrix(1,1) = meanDetectionTimes(1);
    %     detTimeMatrix(1,1) = meanDetectionTimes(1);
    %     detTimeMatrix(1,1) = meanDetectionTimes(1);
    %     detTimeMatrix(1,1) = meanDetectionTimes(1);
    %
    %
    %     plot3(1,3,A(1,1),'kx','MarkerSize',16,'linewidth',2)
    %
    
    
    % %     meanDetectionTimes(2) = meanOmitNaN(detectionTimeMat(2,:),2);
    %     meanDetectionTimes(4) = meanOmitNaN(detectionTimeMat(4,:),2);
    %     l1 = sprintf('Lawnmower (Mean: %3.1f sec., %d/%d valid detections)',meanDetectionTimes(1), numValid(1), numDet(1) );
    % %     l2 = sprintf('Random (Mean: %3.1f sec., %d/%d detections)',meanDetectionTimes(2), numValid(2), numDet(2) );
    %     l4 = sprintf('Likelihood Ratio (Mean: %3.1f sec., %d/%d detections)',meanDetectionTimes(4), numValid(4), numDet(4) );
    %     figure;
    %      subplot(2,1,1)
    %      histogram(detectionTimeMat(1,:),0:5:180)
    %      ylim([0 10])
    %      hold on;
    %      set(gca,'FontSize',16)
    %      legend(l1);
    % %     subplot(3,1,2)
    % %     histogram(detectionTimeMat(2,:),0:5:180)
    % %     ylim([0 10])
    %     ylabel('Number of Occurances')
    % %     set(gca,'FontSize',16)
    % %     legend(l2);
    %      subplot(2,1,2)
    %     histogram(detectionTimeMat(4,:),0:5:180)
    %     ylim([0 10])
    %     legend(l4);
    %     %     histogram(detectionTimeMat(2,:),0:20:300)
    %     %     hold on;
    %
    %     set(gca,'FontSize',16)
    %
    %     %     legend({l1,l2});
    %     xlabel('Detection Time (sec.)')
    %
    %
    
    %     % determine maximum number of pts
    %     maxPts = 0;
    %     timeRange = [];
    %     numTrials = length(alg{algRange(1)}.trial);
    %     for i = algRange
    %         for j = 1:1:numTrials
    %             numPts = length(alg{i}.trial{j}.t);
    %             if ( maxPts < numPts )
    %                 maxPts = numPts;
    %             end
    %         end
    %     end
    %
    %     % create matrix of data for each alg (trials x time)
    %     for i = algRange
    %         alg{i}.totalEntropyMat = NaN*ones(numTrials,maxPts);
    %         alg{i}.discoveredNodePercentageMat = NaN*ones(numTrials,maxPts);
    %         for j = 1:1:numTrials
    %             numPts = length(alg{i}.trial{j}.t);
    %             for k = 1:1:numPts % total target motions
    %                 alg{i}.totalEntropyMat(j,k) = alg{i}.trial{j}.totalEntropy(k);
    %                 %alg{i}.discoveredNodePercentageMat(j,k) = alg{i}.trial{j}.discoveredNodePercentage(k);
    %             end
    %
    %
    %         end
    %     end
    
    %         % visual inspection: plot all data
    %         for i = algRange
    %             for j = 1:1:numTrials
    %                 figure(10)
    %                 plot(alg{i}.trial{j}.discoveredNodePercentage,'o','linewidth',2)
    %                 hold on;
    %             end
    %         end
    %
    
    %         % generate plots of mean/stdev of discoveredNodePercentage
    %         figure;
    %         alpha = 0.25;
    %         timeRange = [1:1:maxPts];
    %         [plotHandle1] = plot(timeRange, meanOmitNaN(alg{1}.discoveredNodePercentageMat,1), 'k','linewidth',2);
    %         hold on;
    %         [plotHandle2] = plotMonteCarlo(timeRange, alg{2}.discoveredNodePercentageMat, 'r', alpha);
    %         [plotHandle3] = plotMonteCarlo(timeRange, alg{4}.discoveredNodePercentageMat, 'b', alpha);
    %         xlabel('Time (sec)')
    %         ylabel('Percent Nodes Discovered')
    %         set(gca,'FontSize',16)
    %         axis tight;
    %         ylim([0 100]);
    %         legend([plotHandle1 plotHandle2 plotHandle3],{algName,'Random Wpts','Mutual Info.'})
    %
    %         % generate plots of mean/stdev of discoveredNodePercentage
    %         figure;
    %         alpha = 0.25;
    %         timeRange = [1:1:maxPts];
    %         [plotHandle1] = plot(timeRange, meanOmitNaN(alg{1}.totalEntropyMat,1), 'k','linewidth',2);
    %         hold on;
    %         [plotHandle2] = plotMonteCarlo(timeRange, alg{2}.totalEntropyMat, 'r', alpha);
    %         [plotHandle3] = plotMonteCarlo(timeRange, alg{4}.totalEntropyMat, 'b', alpha);
    %         xlabel('Time (sec)')
    %         ylabel('Entropy (bits)')
    %         set(gca,'FontSize',16)
    %         axis tight;
    %         %ylim([0 100]);
    %         legend([plotHandle1 plotHandle2 plotHandle3],{algName,'Random Wpts','Mutual Info.'})
    %
    % plot coverage
    %     figure;
    % data : is matrix [numRuns x numDataPts]
    %     %
    %     function [meanHandle] = plotMonteCarlo(x, data, c, alpha)
    %     dataMean = mean(data,1);
    %     numRuns = size(data,1);
    %     numDataPts = size(data,2);
    %     dataStdev = std(data,1);
    %     meanHandle = plot(x,dataMean,[c '-'],'linewidth',2);
    %     hold on;
    %     h = fill([x fliplr(x)],[dataMean-dataStdev fliplr(dataMean+dataStdev)],c,'EdgeColor','none');
    %     set(h,'FaceAlpha',alpha);
    %     end
    
end
%
% % then save the data to \results\data, with a brief NOTE string.            %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%