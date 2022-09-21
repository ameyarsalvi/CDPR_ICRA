#!/bin/bash

reward1=(10000 10 100 100 100 100)
filename1=R1
mkdir -p R1 



echo "#!/bin/bash" > job.sh
echo "#PBS -N CRRT1" >> job.sh
echo "#PBS -l select=1:ncpus=36:mem=200gb,walltime=24:00:00" >> job.sh
echo "module add matlab/2022a" >> job.sh
echo "cd /scratch1/adhitir/CDPR_RL/RR_and_fulltraj_with_FOLv2_updated_obj_working" >> job.sh
echo "matlab -nodisplay -r \"CableRobMain [${reward1[@]}] ${filename1}\" > $PWD/${filename1}/CDPR.txt " >> job.sh

qsub job.sh


reward2=(100 100 100 100 100 100)
filename2=R2
mkdir -p R2 

echo "#!/bin/bash" > job.sh
echo "#PBS -N CRRT2" >> job.sh
echo "#PBS -l select=1:ncpus=36:mem=200gb,walltime=24:00:00" >> job.sh
echo "module add matlab/2022a" >> job.sh
echo "cd /scratch1/adhitir/CDPR_RL/RR_and_fulltraj_with_FOLv2_updated_obj_working" >> job.sh
echo "matlab -nodisplay -r \"CableRobMain [${reward2[@]}] $filename2\" > $PWD/${filename2}/CDPR.txt" >> job.sh

qsub job.sh


reward3=(100000 100 100 100 100 100)
filename3=R3
mkdir -p R3 

echo "#!/bin/bash" > job.sh
echo "#PBS -N CRRT3" >> job.sh
echo "#PBS -l select=1:ncpus=36:mem=200gb,walltime=24:00:00" >> job.sh
echo "module add matlab/2022a" >> job.sh
echo "cd /scratch1/adhitir/CDPR_RL/RR_and_fulltraj_with_FOLv2_updated_obj_working" >> job.sh
echo "matlab -nodisplay -r \"CableRobMain [${reward3[@]}] $filename3 \" > $PWD/${filename3}/CDPR.txt " >> job.sh

qsub job.sh

