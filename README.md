# Extended-Kalman-filter
This is a project to implement Extended Kalman filter to track objects.
This Project is the sixth task (Project 1 of Term 2) of the Udacity Self-Driving Car Nanodegree program. The main goal of the project 
is to apply Extended Kalman Filter to fuse data from LIDAR and Radar sensors of a self driving car using C++.

The project was created with the Udacity Starter Code.

## Contents of this repo
* src - Contains all the source files. Files modified as part of implementation are **FusionEKF.cpp**, **Kalman_Filter.cpp**, 
**Kalman_filter.h** and **tools.cpp**.
* build - Contains all the files generated as part of build
* CMakelists.txt

Other folder contents are not modified.

The project uses measurement data from Laser and Radar sensors from the simulator to estimate the position of the object.
The resulting RMSE values are within the threshold requirements of the project.

## Output values of project run on Dataset 1
###RMSE:
**X:** 0.0973
**Y:** 0.0855
**VX:** 0.4513
**VY:** 0.4399

The code has also been run on dataset 2. It is able to successfully track the object with **RMSE** values within threshold.



