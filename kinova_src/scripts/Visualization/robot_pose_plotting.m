%% Robot Pose Plotting

clear all;
close all;
clc;

%% setup robot

% robot_name = 'Kinova_Grasp_URDF.urdf';
% robot_name = 'Kinova_Grasp_Cylinder_Edge.urdf';
robot_name = 'Kinova_Grasp_Gripper_Middle.urdf';
robot = importrobot(robot_name);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];

% get params
add_uncertainty_to = 'all'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

params = load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);

%% Plotting Robot in Specific Pose

% define pose
q_pose = [0;-pi/4;0;-pi/2;0;pi/4;0];

show(robot,q_pose,'Frames','off')
% view(3)
view(-125,37.5)
axis square
grid off