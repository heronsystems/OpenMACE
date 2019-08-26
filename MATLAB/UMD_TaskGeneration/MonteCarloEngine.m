%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A Monte Carlo Engine for main_taskGeneration.m
% S. Cheng, A. Wolek, August 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
format compact;

totalTime = tic;
% function [MonteCarloSwitch] = MonteCarloEngine()
updatePath;

% turn on switch
MonteCarloSwitch = 1;
%rng('default');

% user inputs:
iiIndex = 1:3; % index for algorithm
jjIndex = 1:50; % index for generated scenes (agent initial location and target behavior)
mapIndex = 1:2; % index for maps

parfor jj = 1:1:length(jjIndex) % for target motions
    for m = 1:1:length(mapIndex)
        for for_i = 1:1:length(iiIndex) %  for algorithm
            ii = iiIndex(for_i);
            
            fprintf('Running algorithm %d (trial %d) \n', ii, jj);
            % set the ID of algorithm, initial formation, and target motion 
            % here (remember to load the IDs to the loadParams(ID1,ID2,ID3) 
            % function)
            algorithmID = ii;
            initialFormationID = jj;
            targetMotionID = jj; 
            mapID = m;
            
            % run sim
            main_taskGeneration_parforWrapper(MonteCarloSwitch,algorithmID,initialFormationID,targetMotionID, mapID);
        end
    end
end
fprintf('Total time is %f\n',toc(totalTime));