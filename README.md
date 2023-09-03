# Wrench Analysis for Inertial Transport using Reachability
**Authors:** Zachary Brei (breizach@umich.edu), Jonathan Michaux (jmichaux@umich.edu), Bohao Zhang (jimzhang@umich.edu), Patrick Holmes (pdholmes@umich.edu), and Ram Vasudevan (ramv@umich.edu). 

- All authors are affiliated with the department of Mechanical Engineering and department of Robotics of the University of Michigan, 2505 Hayward Street, Ann Arbor, Michigan, USA.
- This work is supported by ...
- `WAITR` was developed in [Robotics and Optimization for Analysis of Human Motion (ROAHM) Lab](http://www.roahmlab.com/) at University of Michigan - Ann Arbor.

## Introduction (TODO: update to WAITR Abstract)
A key challenge to ensuring the rapid transition of robotic systems from the industrial sector to more ubiquitous applications is the development of algorithms that can guarantee safe operation while in close proximity to humans. Motion planning and control methods, for instance, must be able to certify safety while operating in real-time in arbitrary environments and in the presence of model uncertainty. This paper proposes Wrench Analysis for Inertial Transport using Reachability (WAITR), a certifiably safe motion planning and control framework for serial link manipulators that manipulate unsecured objects in arbitrary environments. WAITR uses reachability analysis to construct over-approximations of the contact wrench applied to unsecured objects, which captures uncertainty in the manipulator dynamics, the object dynamics, and contact parameters such as the coefficient of friction. An optimization problem formulation is presented that can be solved in real-time to generate provably-safe motions for manipulating the unsecured objects. This paper illustrates that WAITR outperforms state of the art methods in a variety of simulation experiments and demonstrates its performance in the real-world.
The link to the project website is https://roahmlab.github.io/waitr-dev/.

## Dependency
The repo has been verified on MATLAB R>=2021b and Ubuntu >= 20.04

This repo depends on the following repos:
 - [CORA](https://tumcps.github.io/CORA/)
 
You need to download this repo and add to your MATLAB path.

This repo assumes that you have installed the following libraries:

 - libboost-dev
 - libeigen3-dev (3.3.7)
 - libipopt
 - libcoinhsl
 
#### Install Boost C++ library
Simply run the following command:

     sudo apt install libboost-dev 

#### Install Eigen3
In this work, we use Boost C++ library for interval arithmetic computation. 
We further put intervals into Eigen library to create interval matrix and interval vector. 
However, currently we only know how to do this for Eigen 3.3.7.
We have not found any solutions to dump Boost intervals to the latest version of Eigen yet.
If you are working in Ubuntu 20.04, the default Eigen library version should be Eigen 3.3.7, so you can install the correct version of Eigen simply by running the following command:

     sudo apt install libeigen3-dev 

If you are working in later version of Ubuntu, you would have to manually install Eigen 3.3.7.
We provide a possible way to do this in the instructions below.
Download `eigen-3.3.7` by following [this link](https://gitlab.com/libeigen/eigen/-/releases/3.3.7).

     cd ~/Downloads
     tar -xvzf eigen-3.3.7.tar.gz
     mv eigen-3.3.7 /your/favorite/path/
     cd /your/favorite/path/eigen-3.3.7
     mkdir build && cd $_
     cmake ..
     sudo make
     sudo make install
 
#### Install Ipopt and HSL (TODO: Maybe add more introduction, but will be fairly long)
libipopt and libcoinhsl could be very annoying to install and to work with MATLAB. 
Suppose libipopt and libcoinhsl are both installed in /usr/local/lib.
You need to add that path to both user's environmental variable 'LD_LIBRARY_PATH' and MATLAB's environment variable 'LD_LIBRARY_PATH'
Check [here](https://www.mathworks.com/help/matlab/matlab_external/set-run-time-library-path-on-linux-systems.html) and [here](https://stackoverflow.com/questions/13428910/how-to-set-the-environmental-variable-ld-library-path-in-linux) for more information.

#### Install submodules
Run
``` sh
git submodule update --init
```

## Building
Run 
 - initialize.m
 - kinova_src/initialize.m
 
in MATLAB before you run any other scripts!

## add what to do in case of error during initialize.m?

## Usage
Before running any scripts, make sure you run the initalization scripts successfully and put the Ipopt libraries in the proper path.

All of our results in the paper is developed based on [Kinova Gen3](https://www.kinovarobotics.com/product/gen3-robots). 
All of the related test scripts are included in [kinova_src](https://github.com/roahmlab/waitr-dev/tree/main/kinova_src).
Check the [README](https://github.com/roahmlab/waitr-dev/blob/main/kinova_src/README.md) in that folder for more information.

## License

`WAITR` is released under a [GNU license](https://github.com/roahmlab/armour-dev/blob/main/LICENSE). For a list of all code/library dependencies, please check dependency section. For a closed-source version of `WAITR` for commercial purpose, please contact the authors.

An overview of the theoretical and implementation details has been published in (TODO: publish in where?). If you use `WAITR` in an academic work, please cite using the following BibTex entry (TODO: fill in reference to our paper):

Note: will add link to paper arxiv version when available.
