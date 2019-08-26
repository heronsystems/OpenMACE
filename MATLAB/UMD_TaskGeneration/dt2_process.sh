#!/bin/tcsh
#SBATCH -t 4:00:00
#SBATCH -n 1
#SBATCH -N 1
#SBATCH -L matlab
#SBATCH --mail-type=ALL
#SBATCH --mail-user=wolek@umd.edu

module load matlab/2018b
matlab -nodisplay -nosplash -r "run('/lustre/wolek/UMD_TaskGeneration/viewMonteCarloResults.m'); exit" > /lustre/wolek/Outputs/dt2_process.out
