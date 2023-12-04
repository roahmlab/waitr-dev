%% user parameters
clear all;
close all;

% filename = 'trial_scene_059_.mat';
filename = ['trial_scene_010_003.csv.mat'];

verbosity = 0 ;
dimension = 3 ;

plot_start_and_end_config_only = true; % otherwise, animate trial.

%% automated from here
load(filename)

u_s = W.u_s;

agent_info = summary.agent_info ;
bounds = summary.bounds ;
obstacles = summary.obstacles ;
planner_info = summary.planner_info ;
start = summary.start ;
goal = summary.goal ;

% agent just for visualizing, parameters may differ
agent_urdf = 'Kinova_Grasp_Cylinder_Edge.urdf';
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot);

% create arm agent
A = uarmtd_agent(robot, params,...
             'verbose', verbosity,...
             'animation_set_axes_flag', 0,... 
             'animation_set_view_flag', 0);

% create world
goal_type = 'configuration';
goal_radius = pi/30;
% W = kinova_grasp_world_static('create_random_obstacles_flag', false, 'include_base_obstacle', true, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
%                             'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles, 'goal_type', goal_type,...
%                             'grasp_constraint_flag', true,'ik_start_goal_flag', true,'u_s', u_s, 'surf_rad', surf_rad) ;

% fill in agent state
A.time = agent_info.time ;
A.state = agent_info.state ;
A.use_CAD_flag = true;
n_links_and_joints = params.nominal.num_joints;
n_states = 2*params.nominal.num_q;
q_index = params.nominal.q_index;

joint_state_indices = 1:2:n_states ;
joint_speed_indices = 2:2:n_states ;
joint_types = params.nominal.joint_types';
joint_axes = params.nominal.joint_axes;

link_shapes = repmat({'cuboid'}, 1, n_links_and_joints);
[link_poly_zonotopes, link_sizes, temp_link_CAD_data] = create_pz_bounding_boxes(robot);
A.load_CAD_arm_patch_data(temp_link_CAD_data)
A.link_plot_edge_color = [1 1 1] ;
A.link_plot_edge_opacity = 0 ;
A.link_plot_face_color = [0.8 0.8 1] ;
A.link_plot_face_opacity = 1 ;
% set up world using arm
W.setup(agent_info)
clear_plot_data(W);

%% Output some key stats

goal_check = summary.goal_check
num_iter = summary.total_iterations
    
%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on ;

plot(W) ;

if dimension == 3
%     view(3)
    view(80,10) ;
    xlim([-0.5 1])
    ylim([-1 1])
    zlim([0 1])
end

if plot_start_and_end_config_only
    plot_at_time(A, 0);
%     grid off
%     axis off
    disp('Press a key to plot final config.');
    pause();
    plot(A) ;
else
    animate(A,'test1.gif');
end

% figure(2)
% plot(A.time,A.reference_acceleration)
% title('Joint Acceleration')
% figure(3)
% plot(A.time,A.state(A.joint_speed_indices,:))
% title('Joint Speeds')