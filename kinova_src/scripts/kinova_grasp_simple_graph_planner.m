%% description
% This script iterates through a list of presaved random worlds and runs
% the ARMOUR planner on them. It then saves information on how well each the
% planner performed in each trial.
%
% Authors: Bohao Zhang (adapted from Patrick Holmes code)
% Created 25 November 2019
% Edited 16 January 2020
% Edited: 02 June 2022 to work with updated UARMTD code
% Edited: 25 September 2022 to work with updated UARMTD code for kinova
% Edited: 10 November 2022 clean up

initialize_script_path = matlab.desktop.editor.getActiveFilename;
cd(initialize_script_path(1:end-35));

close all; clear; clc;

%% user parameters

u_s = 0.6; 
surf_rad =  0.058 / 2;
grasp_constraint_flag = true;

use_robust_input = true;
stop_sim_when_ultimate_bound_exceeded = false;

goal_type = 'configuration'; % pick 'end_effector_location' or 'configuration'
goal_radius = pi/30;
dimension = 3 ;
verbosity = 10;

% trajectory duration (must match C++)
DURATION = 2.0;

%%% for planner
traj_type = 'bernstein'; % pick 'orig' or 'bernstein'
allow_replan_errors = true ;
first_iter_pause_flag = false;
use_q_plan_for_cost = false; % otherwise use q_stop (q at final time)
input_constraints_flag = true;
save_FO_zono_flag = false;

%%% for agent
agent_urdf = 'Kinova_Grasp_w_Tray.urdf';

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
LLC_V_max = 2e-2;
use_true_params_for_robust = false;
if_use_mex_controller = true;
alpha_constant = 10;
Kr = 4;

%%% for HLP
if_use_RRT = false;
HLP_grow_tree_mode = 'new' ;
plot_waypoint_flag = true ;
plot_waypoint_arm_flag  = true ;
lookahead_distance = 0.2 ; % used if RRT is false
increment_waypoint_distance = 0.18;
use_SLP = false; % use SLP between high level graph waypoints

% plotting
plot_while_running = true;

% simulation
max_sim_time = 86400 ; % 24 hours = 86400 sec; 48 hours = sec
max_sim_iter = 1000 ;
stop_threshold = 3 ; % number of failed iterations before exiting

%% Defining Start, Goal and Obstacles

% Hardware start and goal
start = wrapToPi([3.75,-1.0472,0,-2.0944,0,1.5708,0]');
goal = [1.75,-0.5236,0,-2.0944,0,1.0472,0]';
% start_node = 2057743;
% goal_node = 1416420;

% Code Currently Expects 10 Obstacles
obstacles{1} = box_obstacle_zonotope('center', [0.53016; 0.30426; 0.47],...
                                     'side_lengths', [0.24; 0.22; 0.175]) ; % pos: 0.51089; 0.084019; 0.067449
% obstacles{1} = box_obstacle_zonotope('center',[-0.3;0;0.5],...
%                                     'side_lengths',[0.01; 2; 2]);

obstacles{2} = box_obstacle_zonotope('center',[0.53016; 0.30426; 0.13992],...
                                    'side_lengths', [0.32; 0.32; 0.32]);
% obstacles{2} = box_obstacle_zonotope('center',[-0.3;0;0.5],...
%                                     'side_lengths',[0.01; 2; 2]);

% Hardware environment obstacles
% back wall
obstacles{3} = box_obstacle_zonotope('center',[-0.3;0;0.5],...
                                    'side_lengths',[0.01; 2; 2]);
% ceiling
obstacles{4} = box_obstacle_zonotope('center',[0.8; 0; 1],...
                                    'side_lengths',[2; 2; 0.01]);
% side wall
obstacles{5} = box_obstacle_zonotope('center',[1; 1.5; 0],...
                                    'side_lengths',[2; 0.01; 2]);
% floor
obstacles{6} = box_obstacle_zonotope('center',[1; 0; -0.01],...
                                    'side_lengths',[2; 2; 0.01]);

% extra obstacles
% obstacles{6} = obstacles{1};
obstacles{7} = obstacles{1};
obstacles{8} = obstacles{1};
obstacles{9} = obstacles{1};
obstacles{10} = obstacles{1};

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
M_min_eigenvalue = 8.0386; % matlab doesn't import these from urdf so hard code into class

use_cuda_flag = true;

%% automated from here
% run loop
if plot_while_running
    figure(1); clf; view(3); grid on;
end

W = kinova_grasp_world_static('create_random_obstacles_flag', false, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
                        'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles, 'goal_type', goal_type,...
                        'grasp_constraint_flag', true,'ik_start_goal_flag', false, 'u_s', u_s, 'surf_rad', surf_rad) ;

% W.start = [pi/4 - pi; -pi/2; 0; 0; 0; 0; 0];
% W.goal =  [pi / 2; -pi/2; 0; 0; 0; 0; 0];
% for i = 1:length(W.obstacles) 
%     W.obstacles{i}.Z(1,1) = W.obstacles{i}.Z(1,1) + 1000;
% %     W.obstacles{i}.zono.Z(1,1) = W.obstacles{i}.zono.Z(1,1) + 1000;
% end

samplebased_HLP = kinova_samplebased_HLP();

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
                   'increment_waypoint_distance', increment_waypoint_distance,...
                   'use_SLP',use_SLP,...
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
                    'stop_sim_when_ultimate_bound_exceeded', stop_sim_when_ultimate_bound_exceeded) ; 

% %% plotting
if plot_while_running
    figure(1) ; clf ; axis equal ; xlim([-1 1]); ylim([-1 1]); zlim([0 2]); grid on; hold on ;

    if dimension == 3
        view(3);
    end
    
    plot(A);
    plot(W);
end

% for i = 1:size(P.HLP.graph_waypoints,2)
%     A.plot_at_time(P.HLP.graph_waypoints(:,i));
%     pause(0.1);
% end

% run simulation
summary = S.run() ;

%%

figure()
plot(A.time,A.state(A.joint_speed_indices,:))

save('Simulation_GraphOnly_FixedHLP_ObsStackFlip_DifGuess0p25_ID_0p18.mat')