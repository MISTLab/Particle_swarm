#!/bin/bash
#SBATCH --account=def-beltrame
#SBATCH --time=3:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=8

# Stop on any error
set -e

# GUIDE=${1}
# WORKER=${2}
# SEED=${3}

# docker run -it vivekshankarv/hir_benchmark:arm64 run_exp ${1} ${2} ${3} 

singularity run -B /scratch /home/viveks/scratch/Hir/hir.sif run_exp ${1} ${2} ${3} 