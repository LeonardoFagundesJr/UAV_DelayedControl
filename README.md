# UAV Control Analysis Under Delayed Communication
Analysis of the effects of communication time delay in the control of quadrotors 

Programs and additional documentation for the paper "*Communication Delay in UAV Missions: A Controller Gain Analysis to Improve Flight Stability*"

In the paper, a specific positioning task was studied. The implementation of this problem is found in the file `DynCtrl_PosDelayed_256x256AllSims.m`, which can be executed, automatically generating all the graphs and analysis that are present in the paper.

In addition, the work is extended to the study of trajectory tracking tasks, in which there is a time constraint associated with the robot's motion. These analyses are contained in the files `DynCtrl_TrajDelayed_256x256CircSims.m` and `DynCtrl_TrajDelayed_256x256LemnSims.m` which implement trajectories in the shape of a circle and a Bernoulli lemniscate.
