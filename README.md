# CDPR_ICRA
## CDPR Reinforcement Learning Repository for ICRA 2023

### For all cases, run CableRobMain.m file for training and generating the RL agent. Please use Matlab 2022a or above.

### Case A: Tensions only policy 
Cable tensions controlled by reinforcement learning policy for trajectory tracking, no redundancy resolution

<img src="https://github.com/ameyarsalvi/CDPR_ICRA/blob/main/ResultsOnlyT.jpg" width="500" height="400">


### Case B: End-to-end policy
Cable tensions and slider positions controlled by by reinforcement learning policy for trajectory tracking and redundancy resolution, respectively

<img src="https://github.com/ameyarsalvi/CDPR_ICRA/blob/main/ResultsE2E.jpg" width="500" height="400">


### Case C.1: Decoupled policy (Slider positions from optimization) 
Cable tensions controlled by reinforcement learning agent for trajectory tracking, slider positions controlled by optimizer for redundancy resolution

<img src="https://github.com/ameyarsalvi/CDPR_ICRA/blob/main/ResultsOptimT.jpg" width="500" height="400">


### Case C.2: Decoupled policy (Slider positions from slider policy) 
Cable tensions controlled by reinforcement learning agent for trajectory tracking, and slider positions controlled by different reinforcement learning agent for redundancy resolution

<img src="https://github.com/ameyarsalvi/CDPR_ICRA/blob/main/ResultsFinalDecoup.jpg" width="500" height="400">
