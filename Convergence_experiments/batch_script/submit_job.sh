#!/bin/bash

set -e

# Guide number 
# 1,2,3,4
# Worker number
# 10, 50, 100, 250, 500, 1000, 2000, 5000
# Seed
# 30 runs
#
# Launch exploration experiments
#

WORKER=( 10 50 100 250 500 1000 2000 5000 )

# 

for SEED in $(seq 1 30)
do
    for GUIDE in $(seq 1 4)
    do
        for ((WORKER_IDX=0; WORKER_IDX<${#WORKER[*]}; WORKER_IDX++));
        do
            
                while [ $(squeue -u $( whoami ) | wc -l) -ge 990 ]; do sleep 2; done
                WORKER_NUM=${WORKER[WORKER_IDX]}
                RUNID=C_${GUIDE}_${WORKER_NUM}_${SEED}
                echo "Descriptive RUNID: GUide: ${GUIDE} WORKER: ${WORKER_NUM} run: ${SEED}" 
                echo "RUNID: $RUNID" 
                echo "${GUIDE} ${WORKER_NUM} ${SEED}"
                # sbatch --job-name=${RUNID} run_container.sh ${GUIDE} ${WORKER_NUM} ${SEED}
                # sleep 1
            
        done
    done
done