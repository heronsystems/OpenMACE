#!/bin/tcsh
#SBATCH -t 3:00:00
#SBATCH -n 20
#SBATCH -N 1
#SBATCH -L matlab

module load matlab/2018b
matlab -nodisplay -nosplash -r "parpool('local',20);run('/lustre/cheng/UMD_TaskGeneration/MonteCarloEngine.m'); exit" > /lustre/cheng/Outputs/trial.out
