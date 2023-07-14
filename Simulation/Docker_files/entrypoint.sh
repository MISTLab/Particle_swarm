#!/bin/bash

case "$1" in
        run_exp)
            bash /home/docker/Hir/KheperaIV/particle_swarm/Convergence_experiments/batch_script/run_job.sh $2 $3 $4
            ;;
	    exp)
            bash /home/docker/Hir/KheperaIV/ros_simulation/batch_scripts/run_argos.sh $2 $3 $4 $5 $6 $7 $8 $9
            ;;
        exp_bug)
            bash /home/docker/Hir/KheperaIV/ros_simulation/batch_scripts/run_bug_argos.sh $2 $3 $4 $5 $6 $7 $8 $9
            ;;
        exp_hir_bug)
            bash /home/docker/Hir/KheperaIV/ros_simulation/batch_scripts/run_hybrid_argos.sh $2 $3 $4 $5 $6 $7 $8 $9
            ;;
        exp_hetro)
            bash /home/docker/Hir/KheperaIV/ros_simulation/batch_scripts/run_heterogeneous_argos.sh $2 $3 $4 $5 $6 $7 $8 $9
            ;;
        bash)
            /bin/bash
            ;;
        *)
            echo $"Usage: $1 {run_exp | exp| exp_bug | exp_hir_bug | exp_hetro } or exp map_type map conf exp_type"
            exit 1

esac

