#!/bin/bash

#SBATCH -J handGenStartPosition                        # name of job

#SBATCH -p share                                  # name of partition or queue

#SBATCH --array=1-100                     # how many tasks in the array

#SBATCH -o log/LOG-%a.out                 # name of error file for this submission script
#SBATCH -e log/ERROR-%a.err                 # name of error file for this submission script



# load any software environment module required for app (e.g. matlab, gcc, cuda)

#module load software/version

#module load python3

source /nfs/hpc/share/navek/hand-gen-IK/venv/bin/activate




# run my job (e.g. matlab, python)

srun --export=ALL python3 asterisk_IK_paths_fixed.py $SLURM_ARRAY_TASK_ID 100


