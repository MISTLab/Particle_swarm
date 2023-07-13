#!/bin/bash
#SBATCH --account=def-beltrame
#SBATCH --time=12:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=8

# Stop on any error
set -e

# Basic variables
ME=$(whoami)
BASEWORKDIR=${HOME}/scratch/run_dir
HOMEDIR=${HOME}/scratch/SR/KheperaIV
DATADIR=${HOME}/scratch/data
mkdir -p ${DATADIR}

# Other useful variables
#export ARGOS_PLUGIN_PATH=${HOMEDIR}/build
#export LD_LIBRARY_PATH=${ARGOS_PLUGIN_PATH}:$LD_LIBRARY_PATH

# Script parameters
SCRIPT=${1}
SHEEP=${2}
DOGS=${3}
SHAPE=${4}
MOVEMENT=${5}
RANDOMSEED=${6}


# Run id, used as base file name
RUNID=${SHEEP}_${DOGS}_${SHAPE}_${MOVEMENT}_${RANDOMSEED}

echo "${RUNID}"

# Output file names and directories
WORKDIR=${BASEWORKDIR}/${ME}_${RUNID}
OUTFILE=dogVars_${RUNID}.csv
POSFILE=pos_${RUNID}.csv
EXPERIMENT=run_${RUNID}.argos

# Create directory
rm -rf ${WORKDIR}
mkdir -p ${WORKDIR}
mkdir -p ${WORKDIR}/buzz_scripts
mkdir -p ${WORKDIR}/loop_funcs/build
cd ${WORKDIR}

# Copy script files # REVIEW removed since they'll just be there anyway?
cp ${HOMEDIR}/buzz_scripts/Object_movement_test.bo ${WORKDIR}/buzz_scripts
cp ${HOMEDIR}/buzz_scripts/Object_movement_test.bdb ${WORKDIR}/buzz_scripts
cp ${HOMEDIR}/loop_funcs/build/libkh_exp_lf.so ${WORKDIR}/loop_funcs/build/


# Set up experiment file
sed -e "s|RANDOMSEED|${RANDOMSEED}|g" \
    -e "s|DOGS_NUM|${DOGS}|g" \
    -e "s|SHEEP_NUM|${SHEEP}|g" \
    -e "s|SHAPE_NAME|${SHAPE}|g" \
    -e "s|MOVEMENT|${MOVEMENT}|g" \
    -e "s|OUTFILE|${OUTFILE}|g" \
    -e "s|POSFILE|${POSFILE}|g" \
    ${HOMEDIR}/template.argos > ${EXPERIMENT}

# Launch ARGoS
time argos3 -c ${EXPERIMENT}

# Copy files back to the data directory
cp -af ${OUTFILE} ${DATADIR}
cp -af ${POSFILE} ${DATADIR}


# Cleanup
# rm -rf ${WORKDIR}
