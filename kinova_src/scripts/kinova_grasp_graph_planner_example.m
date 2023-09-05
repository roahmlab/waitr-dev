%% description
% This script runs WAITR on a random world using a graph based HLP. Must be
% run on a linux system for the HLP to work.
%
% Authors: Zac Brei and Bohao Zhang (adapted from Patrick Holmes code)
% Created 25 November 2019
% Edited 16 January 2020
% Edited: 02 June 2022 to work with updated UARMTD code
% Edited: 25 September 2022 to work with updated UARMTD code for kinova
% Edited: 10 November 2022 clean up

initialize_script_path = matlab.desktop.editor.getActiveFilename;
cd(initialize_script_path(1:end-36));

close all; clear; clc;

%% user parameters

u_s = 0.609382421; 
surf_rad =  0.058 / 2;
grasp_constraint_flag = true;

use_robust_input = true;

goal_type = 'configuration'; % pick 'end_effector_location' or 'configuration'
goal_radius = deg2rad(3);
dimension = 3 ;
verbosity = 10;

% trajectory duration (must match C++)
DURATION = 2;

%%% for planner
traj_type = 'bernstein'; % pick 'orig' or 'bernstein'
allow_replan_errors = true ;
first_iter_pause_flag = false;
use_q_plan_for_cost = false; % otherwise use q_stop (q at final time)
input_constraints_flag = false;
save_FO_zono_flag = false;
use_cuda_flag = false;

%%% for agent
agent_urdf = 'Kinova_Grasp_URDF.urdf';

% RTD-Force Experiment 1: no uncertainty
% RTD-Force Experiment 2: no uncertainty
% RTD-Force Experiment 3: 5% uncertainty in all links (including tray and
%                           object
add_uncertainty_to = 'all'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

agent_move_mode = 'integrator' ; % pick 'direct' or 'integrator'
use_CAD_flag = true;
add_measurement_noise_ = false;
measurement_noise_size_ = 0;

%%% for LLC (must match C++)
LLC_V_max = 1e-2;
use_true_params_for_robust = false;
if_use_mex_controller = false;
alpha_constant = 1;
Kr = 5;

%%% for HLP
if_use_RRT = false;
HLP_grow_tree_mode = 'new' ;
plot_waypoint_flag = true ;
plot_waypoint_arm_flag  = true ;
lookahead_distance = 0.1 ; % used if RRT is false

% plotting
plot_while_running = true ;

% simulation
max_sim_time = 86400 ; % 24 hours = 86400 sec; 48 hours = sec
max_sim_iter = 3000 ;
stop_threshold = 3 ; % number of failed iterations before exiting

% % file handling
% save_file_header = 'trial_' ;
% file_location = '../results/rtd-force/dur2s_largeStateBuffer_10Obs_03082023' ;
% if ~exist(file_location, 'dir')
%     mkdir(file_location);
% end

% for world
num_obstacles = 5;
creation_buffer = 0.075; % meter: buffer from initial/goal configuration

%% robot params:
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);
joint_speed_limits = [-1.3963, -1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218;
                       1.3963,  1.3963,  1.3963,  1.3963,  1.2218,  1.2218,  1.2218]; % matlab doesn't import these from urdf so hard code into class
joint_input_limits = [-56.7, -56.7, -56.7, -56.7, -29.4, -29.4, -29.4;
                       56.7,  56.7,  56.7,  56.7,  29.4,  29.4,  29.4]; % matlab doesn't import these from urdf so hard code into class
transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class
M_min_eigenvalue = 8.2998203638; % matlab doesn't import these from urdf so hard code into class

%% automated from here
% run loop
if plot_while_running
    figure(1); clf; view(3); grid on;
end

% idx = 17:17;

% read world CSV to get start and goal, populate obstacles:
% world_filename = world_file_list(idx).name;
% [start, goal, obstacles] = load_saved_world([world_file_folder world_filename]);

tic;
W = kinova_grasp_world_static('create_random_obstacles_flag', true, 'goal_radius', goal_radius,'N_random_obstacles', num_obstacles, 'N_obstacles', num_obstacles, 'dimension',dimension,'creation_buffer', creation_buffer, 'workspace_goal_check', 0, 'verbose',verbosity, 'goal_type', goal_type, 'grasp_constraint_flag', true,'ik_start_goal_flag', false, 'u_s', u_s, 'surf_rad', surf_rad) ; % 'obstacles', obstacles,length(obstacles), 'start', start, 'goal', goal,
W.robot = robot;

% create arm agent
A = uarmtd_agent(robot, params,...
                 'verbose', verbosity,...
                 'animation_set_axes_flag', 0,... 
                 'animation_set_view_flag', 0,...
                 'move_mode', agent_move_mode,...
                 'use_CAD_flag', use_CAD_flag,...
                 'joint_speed_limits', joint_speed_limits, ...
                 'joint_input_limits', joint_input_limits, ...
                 'add_measurement_noise_', add_measurement_noise_, ...
                 'measurement_noise_size_', measurement_noise_size_,...
                 'M_min_eigenvalue', M_min_eigenvalue, ...
                 'transmision_inertia', transmision_inertia,...
                 't_total', DURATION);

% LLC
if use_robust_input
    A.LLC = uarmtd_robust_CBF_LLC('verbose', verbosity, ...
                                  'use_true_params_for_robust', use_true_params_for_robust, ...
                                  'V_max', LLC_V_max, ...
                                  'alpha_constant', alpha_constant, ...
                                  'Kr', Kr, ...
                                  'if_use_mex_controller', if_use_mex_controller);
else
    A.LLC = uarmtd_nominal_passivity_LLC('verbose', verbosity);
end

A.LLC.setup(A);

P = uarmtd_planner('verbose', verbosity, ...
                   'first_iter_pause_flag', first_iter_pause_flag, ...
                   'use_q_plan_for_cost', use_q_plan_for_cost, ...
                   'input_constraints_flag', input_constraints_flag, ...
                   'use_robust_input', use_robust_input, ...
                   'traj_type', traj_type, ...
                   'use_cuda', use_cuda_flag,...
                   'save_FO_zono_flag', save_FO_zono_flag,...
                   'DURATION',DURATION,...
                   'lookahead_distance',lookahead_distance, ...
                   'plot_HLP_flag', true) ; % _wrapper

P.HLP = kinova_samplebased_HLP();
P.HLP.generatePath(W.obstacles, W.start, W.goal);

% set up world using arm
I = A.get_agent_info ;
W.setup(I) ;
W.bounds = [-1 1 -1 1 0 2];

% place arm at starting configuration
A.state(A.joint_state_indices) = W.start ;

% create simulator
S = simulator_armtd(A,W,P, ...
                    'verbose', verbosity, ...
                    'stop_threshold', stop_threshold, ...
                    'plot_while_running', plot_while_running,...
                    'allow_replan_errors',allow_replan_errors,...
                    'max_sim_time',max_sim_time,...
                    'max_sim_iterations',max_sim_iter,...
                    'stop_sim_when_ultimate_bound_exceeded', use_robust_input) ; 

% %% plotting
if plot_while_running
    figure(1) ; clf ; axis equal ; xlim([-1 1]); ylim([-1 1]); zlim([0 2]); grid on; hold on ;

    if dimension == 3
        view(3);
    end
    
    plot(A);
    plot(W);
end

% run simulation
summary = S.run() ;
