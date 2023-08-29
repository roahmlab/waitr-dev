%% Proccessing ROS Bag
clear all;
close all;
clc;

%% Loading Data
% data = readmatrix("HardwareVideoROSData_04222023.csv");
data = readmatrix("HardwareVideo_MultipleRuns_06_15_2023_ROSBAG_Data.csv");

%% Sorting Data

rosbag.raw_time = data(:,1);
rosbag.frame_id = data(:,2);
rosbag.control_torque = data(:,4:10);
rosbag.pos_track_error = data(:,11:17);
rosbag.vel_track_error = data(:,18:24);
rosbag.debug_pos = data(:,26:32);
rosbag.debug_torque = data(:,33:39);
rosbag.debug_vel = data(:,40:46);
rosbag.debug_q_des = data(:,47:53);
rosbag.debug_qd_des = data(:,54:60);
rosbag.debug_qdd_des = data(:,61:67);
rosbag.debug_time = data(:,68);
rosbag.debug_traj_accel = data(:,69:75);
rosbag.debug_duration = data(:,76);
rosbag.debug_traj_k = data(:,79:85);
rosbag.debug_traj_pos = data(:,86:92);
rosbag.reset = data(:,93);
rosbag.start_time = data(:,94);
rosbag.debug_traj_vel = data(:,96:102);

rosbag.traj_accel = data(:,103:109);
rosbag.traj_accel = rosbag.traj_accel(all(~isnan(rosbag.traj_accel),2),:);
rosbag.traj_duration = data(:,110);
rosbag.traj_duration = rosbag.traj_duration(all(~isnan(rosbag.traj_duration),2));
rosbag.traj_k = data(:,113:119);
rosbag.traj_k = rosbag.traj_k(all(~isnan(rosbag.traj_k),2),:);
rosbag.traj_pos = data(:,120:126);
rosbag.traj_pos = rosbag.traj_pos(all(~isnan(rosbag.traj_pos),2),:);
rosbag.traj_reset = data(:,127);
rosbag.traj_reset = rosbag.traj_reset(all(~isnan(rosbag.traj_reset),2));
rosbag.traj_vel = data(:,130:136);
rosbag.traj_vel = rosbag.traj_vel(all(~isnan(rosbag.traj_vel),2),:);
% rosbag. = data(:,);
% rosbag. = data(:,);
% rosbag. = data(:,);
% rosbag. = data(:,);

%% Saving ROS Bag Data as .mat File

save('HardwareVideo_MultipleRuns_06_15_2023_ROSBAG_Data.mat','rosbag')