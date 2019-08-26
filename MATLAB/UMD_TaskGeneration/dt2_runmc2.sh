#!/bin/tcsh
#SBATCH -t 6:00:00
#SBATCH -n 20
#SBATCH -N 1
#SBATCH -L matlab
#SBATCH --mail-type=ALL
#SBATCH --mail-user=wolek@umd.edu

module load matlab/2018b
matlab -nodisplay -nosplash -r "parpool('local',20);run('/lustre/wolek/UMD_TaskGeneration/MonteCarloEngine2.m'); exit" > /lustre/wolek/Outputs/dt2_runmc2.out
