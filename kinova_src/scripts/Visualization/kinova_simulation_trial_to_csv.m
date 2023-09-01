%% Kinova Simulation Trial to .csv

% This file is the first step in the pipeline for creating a blender
% visual from the results of a WAITR trial.

clear all;
close all;
clc;

%% Load Trial Data File

% filename where data is stored
data_filename = 'trial_scene_010_003.csv';
% load the data into a struct
data = load(data_filename);

%% Extract the Relevant Data for Visualization

% extract the trajectory 
% (note that A.state and A.time have a certain sample rate and the data 
% may need to be interpolated to achieve the framerate desired in blender.
traj = data.A.state(data.A.joint_state_indices,:)';

% extract the obstacles
for i = 1:length(data.W.obstacles)
    obs(i,1:3) = data.W.obstacles{i}.Z(:,1)';
    obs(i,4:6) = [0, 0, 0]; % hardcoded so that orientation of obstacles is axes aligned with world frame
    obs(i,7:9) = 2*[data.W.obstacles{i}.Z(1,2), data.W.obstacles{i}.Z(2,3), data.W.obstacles{i}.Z(3,4)]; % note that pybullet needs the full length so the generators need to be multiplied by 2
end

%% Save the Data in Proper Files

% save the obstacles
obs_save_file = 'obstacles.csv';
writematrix(obs,obs_save_file,'delimiter',',')

% save the trajectory
traj_save_file = 'robot_trajectory.csv';
writematrix(traj,traj_save_file,'delimiter',',')
