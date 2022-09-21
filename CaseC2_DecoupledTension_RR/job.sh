#!/bin/bash
#PBS -N CRRT3
#PBS -l select=1:ncpus=36:mem=200gb,walltime=24:00:00
module add matlab/2022a
cd /scratch1/adhitir/CDPR_RL/RR_and_fulltraj_with_FOLv2_updated_obj_working
matlab -nodisplay -r "CableRobMain [1 2 3 4 5 6] R3 " > /d/Work/Code/CableRobot/CableTensions/Cascaded_RR_train_tensions/R3/CDPR.txt 
