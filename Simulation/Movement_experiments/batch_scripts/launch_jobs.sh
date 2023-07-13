#!/bin/bash

set -e
# ./make_release.sh

# Shape enum
# 1. colver
# 2. dumbbell
# 3. none
# 4. torus
#
# Movement enum
# 1. straight
# 2. diagonal 
# 3. rotation TODO

# Launch shepherding experiments
#

# Other variables in argos. Hardcoded
# isDOGSCluster = true
# shepherdMode = true
# data_size = 100
# density = 0.1
# walls = false
# DOG_START_INDEX = 10000

SCRIPT=Object_movement_test
SHEEP_RANGE=(50 100 300)
DOG_RANGE=(4 8 16) # Must be same length as sheep range
SHAPE_ENUM=(clover dumbbell none torus)
MOVEMENT_ENUM=(straight diagonal rotation)


# Only run straight movement, with clover and dumbbell

for SEED in $(seq 1 30)
do
    for ((SHEEP_IDX=0; SHEEP_IDX<${#SHEEP_RANGE[*]}; SHEEP_IDX++));
    do
        SHEEP="${SHEEP_RANGE[SHEEP_IDX]}"
        # matching dog num to sheep number
        DOGS="${DOG_RANGE[SHEEP_IDX]}"

        # Only run clover and dumbbell
        for SHAPE_IDX in 0 1 # ((SHAPE_IDX=0; SHAPE_IDX<${#SHAPE_ENUM[*]}; SHAPE_IDX++));
        do
            SHAPE=${SHAPE_ENUM[SHAPE_IDX]}

            # Only run straight
            for MOVEMENT_IDX in 0 # ((MOVEMENT_IDX=0; MOVEMENT_IDX<${#MOVEMENT_ENUM[*]}; MOVEMENT_IDX++));
            do
                MOVEMENT=${MOVEMENT_ENUM[MOVEMENT_IDX]}

                RUNID=${SHEEP}_${DOGS}_${SHAPE_IDX}_${MOVEMENT_IDX}_${SEED}
                OUTFILE=dogVars_${RUNID}.csv
                POSFILE=pos_${RUNID}.csv
                echo "Descriptive RUNID: $SCRIPT $SHEEP $DOGS $SHAPE $MOVEMENT $SEED" 
                echo "RUNID: $RUNID" 
                sbatch --job-name=${RUNID} run_job.sh $SCRIPT $SHEEP $DOGS $SHAPE_IDX $MOVEMENT_IDX $SEED
                sleep 1
            done
        done
    done
done

# Diagonal motion. Only clover shape

for SEED in $(seq 1 30)
do
    # Only for 300 sheep
    for SHEEP_IDX in 2 # for ((SHEEP_IDX=0; SHEEP_IDX<${#SHEEP_RANGE[*]}; SHEEP_IDX++));
    do
        SHEEP="${SHEEP_RANGE[SHEEP_IDX]}"
        # matching dog num to sheep number
        DOGS="${DOG_RANGE[SHEEP_IDX]}"

        # Only run clover and dumbbell
        for SHAPE_IDX in 0 # for ((SHAPE_IDX=0; SHAPE_IDX<${#SHAPE_ENUM[*]}; SHAPE_IDX++));
        do
            SHAPE=${SHAPE_ENUM[SHAPE_IDX]}

            # Only run straight
            for MOVEMENT_IDX in 1 # for ((MOVEMENT_IDX=0; MOVEMENT_IDX<${#MOVEMENT_ENUM[*]}; MOVEMENT_IDX++));
            do
                MOVEMENT=${MOVEMENT_ENUM[MOVEMENT_IDX]}

                RUNID=${SHEEP}_${DOGS}_${SHAPE_IDX}_${MOVEMENT_IDX}_${SEED}
                OUTFILE=dogVars_${RUNID}.csv
                POSFILE=pos_${RUNID}.csv
                # if test -f "$POSFILE"; then
                #     if [[ SHEEP -eq 300 ]] && [[ SEED -eq 1 ]]; then
                #         sleep 0.1
                #     else
                echo "Descriptive RUNID: $SCRIPT $SHEEP $DOGS $SHAPE $MOVEMENT $SEED" 
                echo "RUNID: $RUNID" 
                sbatch --job-name=${RUNID} run_job.sh $SCRIPT $SHEEP $DOGS $SHAPE_IDX $MOVEMENT_IDX $SEED
                sleep 1
                #     fi 
                # fi
            done
        done
    done
done

## Shape none with odd number of dogs
DOG_RANGE=(9 9 17)

for SEED in $(seq 1 30)
do
    for ((SHEEP_IDX=0; SHEEP_IDX<${#SHEEP_RANGE[*]}; SHEEP_IDX++));
    do
        SHEEP="${SHEEP_RANGE[SHEEP_IDX]}"
        # matching dog num to sheep number
        DOGS="${DOG_RANGE[SHEEP_IDX]}"

        # Only shape none
        for SHAPE_IDX in 2 # ((SHAPE_IDX=0; SHAPE_IDX<${#SHAPE_ENUM[*]}; SHAPE_IDX++));
        do
            SHAPE=${SHAPE_ENUM[SHAPE_IDX]}

            # Only run straight
            for MOVEMENT_IDX in 0 # ((MOVEMENT_IDX=0; MOVEMENT_IDX<${#MOVEMENT_ENUM[*]}; MOVEMENT_IDX++));
            do
                MOVEMENT=${MOVEMENT_ENUM[MOVEMENT_IDX]}

                RUNID=${SHEEP}_${DOGS}_${SHAPE_IDX}_${MOVEMENT_IDX}_${SEED}
                OUTFILE=dogVars_${RUNID}.csv
                POSFILE=pos_${RUNID}.csv
                echo "Descriptive RUNID: $SCRIPT $SHEEP $DOGS $SHAPE $MOVEMENT $SEED" 
                echo "RUNID: $RUNID" 
                sbatch --job-name=${RUNID} run_job.sh $SCRIPT $SHEEP $DOGS $SHAPE_IDX $MOVEMENT_IDX $SEED
                sleep 1
            done
        done
    done
done
