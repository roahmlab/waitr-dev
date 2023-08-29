initialize_script_path = matlab.desktop.editor.getActiveFilename;
cd(initialize_script_path(1:end-32));

close all; clear; clc;

%% user parameters

u_s = 0.609382421; 
surf_rad =  0.058 / 2;
grasp_constraint_flag = true;

use_robust_input = true;

goal_type = 'configuration'; % pick 'end_effector_location' or 'configuration'
goal_radius = pi/30;
dimension = 3 ;
verbosity = 10;

% trajectory duration (must match C++)
DURATION = 2;

%%% for planner
traj_type = 'bernstein'; % pick 'orig' or 'bernstein'
allow_replan_errors = true ;
first_iter_pause_flag = false;
use_q_plan_for_cost = false; % otherwise use q_stop (q at final time)
input_constraints_flag = true;
save_FO_zono_flag = true;

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
if_use_mex_controller = true;
alpha_constant = 1;
Kr = 5;

%%% for HLP
if_use_RRT = false;
HLP_grow_tree_mode = 'new' ;
plot_waypoint_flag = true ;
plot_waypoint_arm_flag  = true ;
lookahead_distance = 0.1 ;

% plotting
plot_while_running = true ;

% simulation
max_sim_time = 86400 ; % 24 hours = 86400 sec; 48 hours = sec
max_sim_iter = 3000 ;
stop_threshold = 3 ; % number of failed iterations before exiting

% world file
world_file_header = 'scene';
world_file_folder = '../../saved_worlds/rtd-force/dur2s_largeStateBuffer_10Obs_03082023/';
world_file_location = sprintf('%s*%s*', world_file_folder, world_file_header);
world_file_list = dir(world_file_location);

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

use_cuda_flag = true;

%% perform collision checking
idx = 87;

% parameters
% have to be consistent with FastCollisionChecking.h
NUM_NODES = 100;

% read world CSV to get start and goal, populate obstacles:
world_filename = world_file_list(idx).name;
[start, goal, obstacles] = load_saved_world([world_file_folder world_filename]);

W = kinova_grasp_world_static('create_random_obstacles_flag', false, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
                        'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles, 'goal_type', goal_type,...
                        'grasp_constraint_flag', true,'ik_start_goal_flag', true, 'u_s', u_s, 'surf_rad', surf_rad) ;

return;

% obstacle number hardcoded as 10
Zs = [];
for i = 1:10
    Zs = [Zs; obstacles{i}.Z'];
end

% we have 8 links here
% provide 8 joint position + end effector position
JP = zeros(9,3);
for i = 1:7
    T = forward_kinematics(W.start(1:i), params.nominal.T0, params.nominal.joint_axes);
    JP(i,:) = T(1:3,4);
end
T = forward_kinematics([W.start(1:i);0], params.nominal.T0, params.nominal.joint_axes);
JP(8,:) = T(1:3,4);
T = forward_kinematics([W.start(1:i);0;0;0], params.nominal.T0, params.nominal.joint_axes);
JP(9,:) = T(1:3,4);

% just repeat with the same joint positions for testing
joint_positions = [];
for i = 1:NUM_NODES
    joint_positions = [joint_positions; JP];
end

writematrix(joint_positions, 'joint_positions.csv', 'Delimiter', ' ');
writematrix(Zs, 'obstacles.csv', 'Delimiter', ' ');

% call collision checker in CUDA
system('./collision_checker');

%% verification
link_c = readmatrix('link_c.csv');
link_c = reshape(link_c, [10, NUM_NODES, 8]); % obstacle index, node index, link index

% just choose one of the node since they are the same
link_c = squeeze(link_c(:,777,:));

disp('difference with Matlab results');
for o = 1:10
    for i = 1:8
        % link zonotope (1D)
        Z_c = 0.5 * (JP(i,:) + JP(i+1,:));
        Z_g = 0.5 * (JP(i,:) - JP(i+1,:));
    
        buffered_Z = [obstacles{o}.Z, Z_g'];
    
        [PA, Pb] = polytope_PH(buffered_Z);
    
        c = -max(PA * Z_c' - Pb);

        disp(link_c(o,i) - c);
    end
end
