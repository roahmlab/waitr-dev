%% Plotting ROS Bag Data
clear all;
close all;
clc;

%% To Do

% plot q_des vs index to find the indices that jumps occur
% adjust the positions after the jumps
% recalculate the position tracking error
% plot the new position tracking error

%% Loading ROS Bag Data

load('HardwareVideoROSData.mat');

%% Adjusting Time to Zero

time = rosbag.raw_time(:) - rosbag.raw_time(1);

%% Plotting

figure()
subplot(2,1,1)
hold on
% position tracking error
plot(time,rosbag.pos_track_error)
% position ultimate bound
plot([time(1) time(end)],[0.0132 0.0132],'--r')
plot([time(1) time(end)],[-0.0132 -0.0132],'--r')
subplot(2,1,2)
hold on
% velocity tracking error
plot(time,rosbag.vel_track_error)
% velocity ultimate bound
plot([time(1) time(end)],[0.132 0.132],'--r')
plot([time(1) time(end)],[-0.132 -0.132],'--r')

figure()
plot(rosbag.debug_q_des)
