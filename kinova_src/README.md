# Test Scripts / Interfaces For Kinova Gen3

## Start
   
Please run 'kinova_src/initialize.m' before running any other scripts!

## Test Scripts
This folder holds the different scripts for running WAITR trials.

 - scripts/kinova_simple_example.m: Test WAITR on a user defined scenario.
 - scripts/kinova_simple_example.m: Test WAITR on a random scenario with a user-specified number of obstacles.
 - scripts/kinova_create_random_worlds.m: Create a user-specified number of scenarios with random obstacles.
 - scripts/kinova_run_100_worlds.m: Test WAITR on user-specified number scenarios from a user-specified path to a folder with pre-created scenarios.
 - scripts/kinova_test_summary.m: Return a summary of the test results of a folder which stores simulation results.

## Visualization Scripts

This folder holds different scripts for visualizing trajectories, constraints and the robot. 

 - scripts/visualization/kinova_replay_trial.m: Replay a WAITR trial from saved data.

## Data Processing

This folder holds some MATLAB scripts used to process data collected using ROS on the hardware set up.
