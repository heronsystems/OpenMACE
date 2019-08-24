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
algIndex = 1:3; % index for algorithm
agentInitIndex = 1:4; % index for generated scenes (agent initial location and target behavior)
mapIndex = 1:3; % index for maps

totalTime=  tic;
for initialFormationID = 1:1:length(agentInitIndex) % for target motions
    for mapID = 1:1:length(mapIndex)
        for algorithmID = 1:1:length(algIndex) %  for algorithm                                                            
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
                
                clearvars -except totalTime algIndex agentInitIndex mapIndex processingType algorithmID mapID initialFormationID MonteCarloSwitch detection detectionValid detectionTime alg;
            end
            
        end
    end
end
fprintf('Total time is %f\n',toc(totalTime));
if strcmp(processingType,'analysis')
save('MonteCarloData_processed.mat')
end