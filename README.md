![](https://github.com/roahmlab/armour-dev/blob/main/assets/armour_logo.png?raw=true)

# Autonomous Robust Manipulation via Optimization with Uncertainty-aware Reachability
**Authors:** Jonathan Michaux (jmichaux@umich.edu), Patrick Holmes (pdholmes@umich.edu), Bohao Zhang (jimzhang@umich.edu), Che Chen (cctom@umich.edu), Baiyue Wang (baiyuew@umich.edu), Shrey Sahgal (shreyps@umich.edu), Tiancheng Zhang (zhangtc@umich.edu), Sidhartha Dey (sid.dey@agilityrobotics.com), Shreyas Kousik (skousik@gatech.edu), and Ram Vasudevan (ramv@umich.edu). 

- All authors are affiliated with the Robotics Institute and department of Mechanical Engineering of the University of Michigan, 2505 Hayward Street, Ann Arbor, Michigan, USA (TODO: include Shreyas and Sid).
- This work is supported by the Ford Motor Company via the Ford-UM Alliance under award N022977, National Science Foundation Career Award 1751093 and by the Office of Naval Research under Award Number N00014-18-1-2575.
- `ARMOUR` was developed in [Robotics and Optimization for Analysis of Human Motion (ROAHM) Lab](http://www.roahmlab.com/) at University of Michigan - Ann Arbor.

## Introduction (TODO: I just put abstract here for now)
A key challenge to the widespread deployment of robotic manipulators is the need to ensure safety in arbitrary environments while generating new motion plans in real-time.
In particular, one must ensure that the manipulator does not collide with obstacles, collide with itself, or exceed its own joint torque limits.
This challenge is compounded by the need to account for uncertainty in the mass and inertia of manipulated objects, and potentially the robot itself.
The present work addresses this challenge by proposing Autonomous Robust Manipulation via Optimization with Uncertainty-aware Reachability (`ARMOUR`), a provably-safe, receding-horizon trajectory planner and tracking controller framework for serial link manipulators.
In particular, this paper makes three contributions.
First, a robust, passivity-based controller enables a manipulator to track desired trajectories with bounded error despite uncertain dynamics.
Second, a novel variation on the Recursive Newton-Euler Algorithm (RNEA) allows \methodname to compute the set of all possible inputs required to track any trajectory within a continuum of desired trajectories.
Third, this paper provides a method to compute the swept volume of the manipulator given a reachable set of states; this enables one to guarantee safety by checking that the swept volume does not intersect with obstacles.
The proposed method is compared to state of the art methods and demonstrated on a variety of challenging manipulation examples in simulation, such as maneuvering a heavy dumbbell with uncertain mass around obstacles. 
The link to the project website is (TODO: make a github website as well).

<img height="270" src="/figures/urmtd_front_figure.png"/>

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

## Usage
Before running any scripts, make sure you run the initalization scripts successfully and put the Ipopt libraries in the proper path.

All of our results in the paper is developed based on [Kinova Gen3](https://www.kinovarobotics.com/product/gen3-robots). 
All of the related test scripts are included in [kinova_src](https://github.com/roahmlab/armour-dev/tree/main/kinova_src).
Check the [README](https://github.com/roahmlab/armour-dev/blob/main/kinova_src/README.md) in that folder for more information.

## License

`ARMOUR` is released under a [GNU license](https://github.com/roahmlab/armour-dev/blob/main/LICENSE). For a list of all code/library dependencies, please check dependency section. For a closed-source version of `ARMOUR` for commercial purpose, please contact the authors. (TODO: Maybe not this type of license)

An overview of the theoretical and implementation details has been published in (TODO: publish in where?). If you use `ARMOUR` in an academic work, please cite using the following BibTex entry (TODO: fill in reference to our paper):

      @article{9792203,
            author={Ewen, Parker and Li, Adam and Chen, Yuxin and Hong, Steven and Vasudevan, Ram},
            journal={IEEE Robotics and Automation Letters}, 
            title={These Maps are Made for Walking: Real-Time Terrain Property Estimation for Mobile Robots}, 
            year={2022},
            volume={7},
            number={3},
            pages={7083-7090},
            doi={10.1109/LRA.2022.3180439}}
