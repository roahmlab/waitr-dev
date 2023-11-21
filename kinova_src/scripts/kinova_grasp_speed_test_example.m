%% description
% This script defines a simple world with user defined obstacles and runs
% the ARMOUR planner on them. It then saves information on how well each the
% planner performed in each trial.
%
% Authors: Bohao Zhang (adapted from Patrick Holmes code)
% Created 25 November 2022

initialize_script_path = matlab.desktop.editor.getActiveFilename;
cd(initialize_script_path(1:end-33));

close all; clear; clc; dbstop if error

delete(gcp('nocreate'))
% parpool('threads')

%% user parameters
goal_type = 'configuration'; % pick 'end_effector_location' or 'configuration' or 'fk_func'
goal_radius = pi/30;
dimension = 3 ;
verbosity = 10;

DURATION = 2.0;

u_s = 0.6;
surf_rad =  0.058 / 2;

%%% for planner
traj_type = 'bernstein'; % pick 'orig' (ARMTD) or 'bernstein' (ARMOUR)
use_cuda_flag = true;
grasp_constraints_flag = true;
input_constraints_flag = true;

%%% for agent
agent_urdf = 'Kinova_Grasp_w_Tray.urdf';

add_uncertainty_to = 'link'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {'shoulder_link','half_arm_1_link','half_arm_2_link','forearm_link','spherical_wrist_1_link','spherical_wrist_2_link','bracelet_link','gripper_base','waiter_tray_link','object_link'}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.95, 1.05];
specific_uncertain_mass_range = [0.98, 1.02;
                                0.98, 1.02;
                                0.98, 1.02;
                                0.98, 1.02;
                                0.98, 1.02;
                                0.98, 1.02;
                                0.98, 1.02;
                                0.98, 1.02;
                                0.98, 1.02;
                                0.95, 1.05];

agent_move_mode = 'integrator' ; % pick 'direct' or 'integrator'
use_CAD_flag = true; % plot robot with CAD or bounding boxes

%%% for LLC
use_robust_input = true;
use_true_params_for_robust = false;
if_use_mex_controller = true;
LLC_V_max = 2e-2;
alpha_constant = 10;
Kr = 4.0;

%%% for HLP
% default is a straight-line planner
if_use_RRT = false; % use an RRT HLP
if_use_graph_HLP = false; % use a graph HLP (switch to graph example file to use)
% for RRT
HLP_grow_tree_mode = 'new' ; % pick 'new' or 'keep'
plot_waypoint_flag = true ;
plot_waypoint_arm_flag  = true ;
% for SLP and Graph
lookahead_distance = 0.5 ;
% for Graph
increment_waypoint_distance = pi/30;

% plotting
plot_while_running = true ;

% simulation
max_sim_time = 172800 ; % 48 hours
max_sim_iter = 250 ;
stop_threshold = 3 ; % number of failed iterations before exiting

%%% for world
% start = [-1; -1; -1; -1; -1; -1; -1]; % start configuration
% goal = [1; 1; 1; 1; 1; 1; 1]; % goal configuration

% simple rotation
% goal = [-pi/3;-pi/2;0;0;0;0;0];
% start = [pi/3;-pi/2;0;0;0;0;0];

% start = [-pi/6;-pi/2;-pi/2;pi/2;0;pi/2;pi/2];
% goal = [pi/6;-pi/2;pi/2;pi/2;pi;-pi/2;pi/2];

% start = [3.9270, -1.0472, 0, -2.0944, 0, 1.5708, 0]';
start = [3.9270, -1.0472, 0, -2.0944, 0, 1.5708, 0]';
goal = [2.5,-0.5236,0,-2.0944,0,1.0472,0]';

% start = [0.4933;
%     0.9728;
%     0.6090;
%    -0.3981;
%     0.4258;
%    -1.5576;
%     0.7482];
% goal = [0.3808;
%     1.8013;
%     0.8670;
%    -0.7256;
%     0.7211;
%    -2.0458;
%     1.4523];


% swing
% start = [0;-pi/2;0;0;0;0;0];
% goal = [pi;-pi/2;pi;0;0;0;0];

% random that struggles to reach goal
% use to debug gradients as well
% start = [0.9534;-1.4310;0.1330;0.6418;-0.9534;-0.9534;0.0637];
% goal = [1.62310000000000;-1.59990000000000;-0.137000000000000;0.493080000000000;-3.26490000000000;-2.23000000000000;-0.246620000000000];

% obstacles{1} = box_obstacle_zonotope('center', [3; 3; 3],...
%                                      'side_lengths', [0.1; 0.1; 0.1]) ;
% obstacles{2} = box_obstacle_zonotope('center', [0.3; 0; 0.4],...
%                                      'side_lengths', [0.1; 0.8; 0.05]) ;

% Hardware Obstacles
% box obstacle 1
obstacles{1} = box_obstacle_zonotope('center', [0.53016; 0.10426; 0.1],... % z=0.13992
                                     'side_lengths', [0.24; 0.22; 0.175]) ; % pos: 0.51089; 0.084019; 0.067449
% obstacles{1} = box_obstacle_zonotope('center',[-0.3;0;0.5],...
%                                     'side_lengths',[0.01; 2; 2]);
% box obstacle 2
obstacles{2} = box_obstacle_zonotope('center',[0.53016; 0.10426; 0.1],...
                                    'side_lengths', [0.32; 0.32; 0.32]);
% obstacles{2} = box_obstacle_zonotope('center',[-0.3;0;0.5],...
%                                     'side_lengths',[0.01; 2; 2]);
% wall obstacles
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

%% robot params:
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range,...
                           'specific_uncertain_mass_range',specific_uncertain_mass_range);
joint_speed_limits = [-1.3963, -1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218;
                       1.3963,  1.3963,  1.3963,  1.3963,  1.2218,  1.2218,  1.2218]; % matlab doesn't import these from urdf so hard code into class
joint_input_limits = [-56.7, -56.7, -56.7, -56.7, -29.4, -29.4, -29.4;
                       56.7,  56.7,  56.7,  56.7,  29.4,  29.4,  29.4]; % matlab doesn't import these from urdf so hard code into class
transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class
M_min_eigenvalue = 8.29938; % 8.0386472; % ; matlab doesn't import these from urdf so hard code into class

% figure(101)
% show(robot)

%% automated from here
if plot_while_running
    figure(1); clf; view(3); grid on;
end

% run loop
tic;
W = kinova_grasp_world_static('create_random_obstacles_flag', false,...
                                'goal_radius', goal_radius,...
                                'dimension',dimension,...
                                'workspace_goal_check', 0,...
                                'verbose',verbosity,...
                                'start', start,...
                                'goal', goal,...
                                'goal_type', goal_type, ...
                                'grasp_constraint_flag', grasp_constraints_flag,...
                                'ik_start_goal_flag', true,...
                                'u_s', u_s, ...
                                'surf_rad', surf_rad,...
                                'robot_params', params); % ,... 
                                % 'N_obstacles',length(obstacles),...
                                % 'obstacles', obstacles);

% create arm agent
A = uarmtd_agent(robot, params,...
                 'verbose', verbosity,...
                 'animation_set_axes_flag', 0,... 
                 'animation_set_view_flag', 0,...
                 'move_mode', agent_move_mode,...
                 'use_CAD_flag', use_CAD_flag,...
                 'joint_speed_limits', joint_speed_limits, ...
                 'joint_input_limits', joint_input_limits, ...
                 'add_measurement_noise_', false, ...
                 'measurement_noise_size_', 0,...
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
                   'first_iter_pause_flag', false, ...
                   'use_q_plan_for_cost', true, ...
                   'input_constraints_flag', input_constraints_flag, ...
                   'grasp_constraints_flag', grasp_constraints_flag,...
                   'use_robust_input', use_robust_input, ...
                   'traj_type', traj_type, ...
                   'use_cuda', use_cuda_flag,...
                   'plot_HLP_flag', true, ...
                   'lookahead_distance', lookahead_distance, ...
                   'increment_waypoint_distance',increment_waypoint_distance,...
                   'u_s', u_s,...
                   'surf_rad', surf_rad,...
                   'DURATION', DURATION) ; % 't_move_temp', t_move't_plan', t_plan,...'t_stop', t_stop % _wrapper

if if_use_RRT
    P.HLP = arm_end_effector_flat_RRT_star_HLP('plot_waypoint_flag',plot_waypoint_flag,...
                                               'plot_waypoint_arm_flag',plot_waypoint_arm_flag,...
                                               'grow_tree_mode',HLP_grow_tree_mode,...
                                               'buffer',0.1) ;
end

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
                    'allow_replan_errors',true,...
                    'max_sim_time',max_sim_time,...
                    'max_sim_iterations',max_sim_iter,...
                    'stop_sim_when_ultimate_bound_exceeded', false) ; 

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
summary = S.run();

%% Plotting States

figure(101)
subplot(3,1,1)
title('Joint Positions')
plot(A.time,A.state(A.joint_state_indices,:))
subplot(3,1,2)
title('Joint Velocities')
plot(A.time,A.state(A.joint_speed_indices,:))
subplot(3,1,3)
title('Joint Torques')
plot(A.time,A.input)

% Calculating the Acceleration

joint_angles = A.state(A.joint_state_indices,:);
joint_angular_velocity = A.state(A.joint_speed_indices,:);

qdd_post = zeros(7,length(A.time));
% calculating the acceleration in post to compare with what is stored
for i = 2:length(A.time)
    [M, C, g] = A.calculate_dynamics(joint_angles(:,i), joint_angular_velocity(:,i), A.params.true);

    for j = 1:A.n_inputs
        M(j,j) = M(j,j) + A.transmision_inertia(j);
    end
    % can I call u=A.LLC.get_control_inputs() here with the P.info?
    
    qdd_post(:,i) = M\(A.input(:,i)-C*joint_angular_velocity(:,i)-g);
%         qdd_post(:,i) = M\(A.input(:,i)-C*joint_angular_velocity(:,i)-g);

end

figure(1001)
hold on
title('Acceleration')
% plot(A.time,qdd_post,'o')
plot(A.time,A.reference_acceleration)

% Calling RNEA

for i = 1:length(A.time)

    % clear relevant variables
    clear tau f n

    % call rnea
    [tau, f, n] = rnea(joint_angles(:,i)',joint_angular_velocity(:,i)',joint_angular_velocity(:,i)',qdd_post(:,i)',true,A.params.true); % A.acceleration(:,i)'
%     [tau, f, n] = rnea(joint_angles(:,i)',joint_angular_velocity(:,i)',joint_angular_velocity(:,i)',accel_short(:,i)',true,A.params.true); % A.acceleration(:,i)'

    % store rnea results
    Tau{i} = tau;
    F{i} = f;
    N{i} = n;

    % store the contact forces
    force(:,i) = f(:,10);
    % store the contact moments
    moment(:,i) = n(:,10);

end

% Plotting the forces

% if plot_force

    figure(3)
    % plot the x-axis force (in the tray frame)
    subplot(1,3,1)
    hold on
    plot(A.time(1:end), force(1,:), 'k')
    xlabel('time (s)')
    ylabel('x-axis Force (N)')
    axis('square')
    grid on
    % plot the y-axis force (in the tray frame)
    subplot(1,3,2)
    hold on
    plot(A.time(1:end), force(2,:), 'k')
    xlabel('time (s)')
    ylabel('y-axis Force (N)')
    axis('square')
    grid on
    % plot the z-axis force (in the tray frame)
    subplot(1,3,3)
    hold on
    plot(A.time(1:end), force(3,:), 'k')
    xlabel('time (s)')
    ylabel('z-axis Force (N)')
    axis('square')
    grid on
    
% end

% Calculating Constraints

% separation constraint
sep = -1*force(3,:);

% slipping constraint
slip = sqrt(force(1,:).^2+force(2,:).^2) - u_s.*abs(force(3,:));
slip2 = force(1,:).^2+force(2,:).^2 - u_s^2.*force(3,:).^2;

for i = 1:length(A.time)
    % tipping constraint the normal way
    ZMP_top = cross([0;0;1],moment(:,i)); % normal vector should come first
    ZMP_bottom = dot([0;0;1],force(:,i));
    ZMP(:,i) = ZMP_top/ZMP_bottom;
    ZMP_rad(i) = sqrt(ZMP(1,i)^2+ZMP(2,i)^2);
    tip(i) = ZMP_rad(i) - surf_rad;
    % tipping constraint the PZ way
    ZMP_top2 = cross([0;0;1],moment(:,i));
    ZMP_bottom2 = dot([0;0;1],force(:,i));
    tip2(i) = ZMP_top(1)^2 + ZMP_top(2)^2 - ZMP_bottom^2*(surf_rad)^2;
end

% plotting constraints

% if plot_constraint
    
    figure(4)
    % plot the separation constraint
    subplot(1,3,1)
    hold on
    plot(A.time(1:end),sep, 'k')
    xlabel('time (s)')
    ylabel('Separation Constraint')
    axis('square')
    grid on
    % plot the slipping constraint
    subplot(1,3,2)
    hold on
    plot(A.time(1:end),slip, 'k')
    plot(A.time(1:end),slip2, 'c')
    xlabel('time (s)')
    ylabel('Slipping Constraint')
    axis('square')
    grid on
    % plot the tipping constraint
    subplot(1,3,3)
    hold on
    plot(A.time(1:end),tip, 'k')
    plot(A.time(1:end),tip2, 'c')
    xlabel('time (s)')
    ylabel('Tipping Constraint')
    axis('square')
    grid on

% end

%% Mean Planning Time
plan_time = [];
for i=1:length(P.info.planning_time)
    plan_time = [plan_time P.info.planning_time{i}];
end
mean_plan_time = mean(plan_time)
max_plan_time = max(plan_time)
iterations = summary.total_iterations
num_brakes = sum(summary.stop_check)

%% Max tilting angle

[max_tilt_angle max_tilt_angle_index] = max(abs(A.state(13,:)))
max_tilt_angle_deg = rad2deg(max_tilt_angle)

% forward_kin = forward_kinematics(A.state(A.joint_state_indices,max_tilt_angle_index),params.true.T0,params.true.joint_axes)
% corresponding_euler_angles = rotm2eul(forward_kin(1:3,1:3))

% %% Max Euler Angles
% 
% for i = 1:length(A.time)
%     forward_kin{i} = forward_kinematics(A.state(A.joint_state_indices,i),params.true.T0,params.true.joint_axes);
%     corresponding_euler_angles = rotm2eul(forward_kin{i}(1:3,1:3));
%     quat_angles{i} = eul2quat(corresponding_euler_angles);
% end
% 
% save('TiltTest_1em4_v2.mat','forward_kin','quat_angles')
velocity = A.state(A.joint_speed_indices,:);
mean_vel = mean(abs(velocity),2);
max_vel = max(abs(velocity),[],2);
velocity_mean_max = [mean_vel max_vel]

%% storing test results

load('Velocity_Krange_Test.mat');
velocity_test_w_krange.test28 = struct();
velocity_test_w_krange.test28.krange = P.jrs_info.g_k_bernstein;
velocity_test_w_krange.test28.vel_mean_max = velocity_mean_max;
velocity_test_w_krange.test28.goal_check = summary.goal_check;
velocity_test_w_krange.test28.planning_time = summary.planning_time;
velocity_test_w_krange.test28.u_s = u_s;
save('Velocity_Krange_Test.mat','velocity_test_w_krange')