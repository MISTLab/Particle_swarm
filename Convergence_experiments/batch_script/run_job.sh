#!/bin/bash
#SBATCH --account=def-beltrame
#SBATCH --time=3:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=8

# Basic variables
ME=viveks #$(whoami)
BASEWORKDIR=/scratch/${ME}/Hir/run_dir
HOMEDIR=/home/docker/Hir/KheperaIV/particle_swarm/Convergence_experiments
DATADIR=/scratch/${ME}/Hir/data
mkdir -p ${DATADIR}
mkdir -p ${BASEWORKDIR}


# Other useful variables

# Script parameters

NUMBER_OF_GUIDE_ROBOTS=${1}
NUMBER_OF_WORKER_ROBOTS=${2}
RANDOMSEED=${3}

FILE_MAP_TYPE=${MAP_TYPE//\//_}

# Run id, used as base file name
RUNID=C_${NUMBER_OF_GUIDE_ROBOTS}_${NUMBER_OF_WORKER_ROBOTS}_${RANDOMSEED}

echo "${RUNID}"

# Stop on any error
set -e


# Output file names and directories
WORKDIR=${BASEWORKDIR}/${ME}_${RUNID}
OUTFILE=${RUNID}
EXPERIMENT=run_${RUNID}.argos

# Create directory
rm -rf ${WORKDIR}
mkdir -p ${WORKDIR}
# mkdir -p ${WORKDIR}/buzz_scripts
# mkdir -p ${WORKDIR}/loop_funcs/build
cd ${WORKDIR}

# Copy script files # REVIEW removed since they'll just be there anyway?
# cp ${HOMEDIR}/buzz_scripts/Object_movement_test.bo ${WORKDIR}/buzz_scripts
# cp ${HOMEDIR}/buzz_scripts/Object_movement_test.bdb ${WORKDIR}/buzz_scripts
# cp ${HOMEDIR}/loop_funcs/build/libkh_exp_lf.so ${WORKDIR}/loop_funcs/build/


# Set up experiment file
sed -e "s|RANDOMSEED|${RANDOMSEED}|g" \
    -e "s|NUMBER_OF_GUIDE_ROBOTS|${NUMBER_OF_GUIDE_ROBOTS}|g" \
    -e "s|NUMBER_OF_WORKER_ROBOTS|${NUMBER_OF_WORKER_ROBOTS}|g" \
    -e "s|OUT_FILE_NAME|${OUTFILE}|g" \
    ${HOMEDIR}/experiments/template.argos > ${EXPERIMENT}



# Launch ARGoS
time argos3 -c ${EXPERIMENT}

# Copy files back to the data directory
cp -af H${OUTFILE}.csv ${DATADIR}
# cp -af ${POSFILE} ${DATADIR}
echo "Copied data"



# Cleanup
rm -rf ${WORKDIR}
