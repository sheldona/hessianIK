# hessianIK
Source code for the paper "Inverse Kinematics Problems with Exact Hessian Matrices" at Motion in Games (MIG) 2017

## License 
Copyright (C) 2017 Kenny Erleben and Sheldon Andrews
Permission to use and modify in any way, and for any purpose, this
software, is granted by the author.  Permission to redistribute
unmodified copies is also granted.  Modified copies may only be
redistributed with the express written consent of Kenny Erleben
(kenny@di.ku.dk) or Sheldon Andrews (sheldon.andrews@gmail.com)

## Python code
The 'python' folder contains a Python implementation of the framework described in the paper.

The file 'main.py' contains code to verify the algorithms in the paper. 
For the code used in our evaluation and comparisons with mocap data, please see the 'matlab' folder.

## MATLAB code
The 'matlab' folder contains source code used to evaluate exact Hessian methods in 
the paper. Here is a description of individual scripts and functions used in our 
comparisons and evaluations: 

* The top-level folder contains scripts used in our analysis. 
* The 'ik' sub-folder contains functions for computing the objective function, Jacobian, and Hessian, as well as many other helper functions.
* The 'HDM05-Parser' sub-folder contains code provided by Meinard Müller and
colleagues (see http://resources.mpi-inf.mpg.de/HDM05/). Although a few 
small modifications have been made to integrate with our framework.

Please add all sub-folders to your MATLAB path.
  
### Requirements 
MATLAB with the Optimization toolbox installed.
  
### Scripts/functions
* **doIKSolve.m:** Solves walking, kicking, and punching frames from the dg_03-02_01 motion in the HDM05 database. The motion file may be edited by changing the readMocap() lines near the top of the script. Other parameters to change/experiment with include *maxIter*, *tol*, *method*, and *useLimits*.

* **doReconstructMotion.m:** Reconstructs the selected motion. Edit the readMocap() lines near the top of the script to change the motion. This script was used to generate the motion sequences in the video.
  
* **doParamSweep.m:** Perform a parameter sweep of preconditioned conjugate gradient (CG) 
  parameters. This script was used to generate the plot in Figure 5.
  
**ik/runIK.m:** The main IK optimization function. This select between the quasi-Newton and exact Newton optimizer, and applies a few settings from arguments. See documentation near the top of the .m file for more details.
  
* **ik/hessianIK.m:** Computes the Hessian.
  
* **ik/jacobianIK.m:** Computes the Jacobian.
  
* **ik/objectiveIK.m:** Computes the objective function.
  
* **ik/computeBounds.m:** Compute lower and upper bounds on joint angles from motion data. Note that the limits in the ASF file are only really used for editing software (as noted in [Müller et al., 2007]). The solved motion violates the  limits regularly it seems. So here we estimate limits from existing motion, plus a little 'wiggle room'.
	 
