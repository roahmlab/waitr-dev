%% description
% This script generates random worlds to be used for evaluating aRmTD
% against other motion planners
%
% Authors: Patrick Holmes
% Created: 11 November 2019

clear ; clc ; figure(1); clf; view(3); grid on;

%% user parameters
world_save_dir = '../saved_worlds/rtd-force/dur2s_largeStateBuffer_10Obs_Champagne_Second_11262023';
if ~exist(world_save_dir, 'dir')
    mkdir(world_save_dir);
end

u_s = 0.7; 
surf_rad =  0.0685 / 2;
grasp_constraint_flag = true;

transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class
M_min_eigenvalue = 8.00; % 8.29938; % matlab doesn't import these from urdf so hard code into class

obstacle_flag = 1;
N_obstacle_min = 10 ;
N_obstacle_max = 10 ;
N_obstacle_delta = 1 ;
N_worlds_per_obstacle = 100;

dimension = 3 ;
nLinks = 10 ;
verbosity = 10 ;
allow_replan_errors = true ;

t_plan = 1 ;
time_discretization = 0.02 ;
T = 2 ;
DURATION = 2;

use_cuda_flag = true;
agent_move_mode = 'integrator' ; % pick 'direct' or 'integrator'

add_uncertainty_to = 'all'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

% kinova
robot = importrobot('Kinova_Grasp_Champagne_Edge.urdf');
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
% model = create_model_from_urdf('Kinova_Grasp_URDF.urdf');
% model = rmfield(model, 'transmissionInertia');
% model = rmfield(model, 'friction');
% model = rmfield(model, 'damping');
params = load_robot_params(robot,...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);
joint_speed_limits = [-1.3963, -1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218;
                       1.3963,  1.3963,  1.3963,  1.3963,  1.2218,  1.2218,  1.2218]; % matlab doesn't import these from urdf
joint_input_limits = [-56.7, -56.7, -56.7, -56.7, -29.4, -29.4, -29.4;
                       56.7,  56.7,  56.7,  56.7,  29.4,  29.4,  29.4]; % matlab doesn't import these from urdf
A = uarmtd_agent(robot, params, ...
                     'move_mode', agent_move_mode,...
                     'joint_speed_limits', joint_speed_limits, ...
                     'joint_input_limits', joint_input_limits,...
                     'M_min_eigenvalue', M_min_eigenvalue, ...
                     'transmision_inertia', transmision_inertia,...
                     't_total', DURATION);
A.LLC = uarmtd_robust_CBF_LLC();

%% automated from here

test1 = N_obstacle_min:N_obstacle_delta:N_obstacle_max
test2 = 1:N_worlds_per_obstacle

if obstacle_flag
    for i = N_obstacle_min:N_obstacle_delta:N_obstacle_max
        for j = 1:N_worlds_per_obstacle
        
            % use this to start from random start config:
            W = kinova_grasp_world_static('robot',robot,'include_base_obstacle', 1, 'goal_radius', pi/30, 'N_random_obstacles',i,'dimension',dimension,'workspace_goal_check', 0,...
                'verbose',verbosity, 'creation_buffer', 0.075, 'base_creation_buffer', 0.075,...
                'grasp_constraint_flag', grasp_constraint_flag,'ik_start_goal_flag', false,'u_s', u_s, 'surf_rad', surf_rad) ;
    
            % set up world using arm
            I = A.get_agent_info ;
            W.setup(I)
    
            % place arm at starting configuration
            A.state(A.joint_state_indices) = W.start ;
            
            filename = sprintf('%s/scene_%03d_%03d.csv', world_save_dir, i, j);
    
            % create .csv file
            write_fetch_scene_to_csv(W, filename);
    
        end
    end
else
    for j = 1:N_worlds_per_obstacle
    
        % use this to start from random start config:
        W = kinova_grasp_world_static('robot',robot,'include_base_obstacle', 1, 'goal_radius', pi/30,'dimension',dimension,'workspace_goal_check', 0,...
            'verbose',verbosity, 'creation_buffer', 0.075, 'base_creation_buffer', 0.075,...
            'grasp_constraint_flag', grasp_constraint_flag,'ik_start_goal_flag', false,'u_s', u_s, 'surf_rad', surf_rad) ;

        % set up world using arm
        I = A.get_agent_info ;
        W.setup(I)

%         W.start = [0;-pi/2;0;0;0;0;0];
        % place arm at starting configuration
%         A.state(A.joint_state_indices) = W.start ;

        % set up world using arm
        I = A.get_agent_info ;
%         W.setup(I) ;
        W.bounds = [-1 1 -1 1 0 2];
%         if dimension == 3
%             view(3);
%         end
%         
%         plot(A);
%         plot(W);
        
        filename = sprintf('%s/scene_%03d_%03d.csv', world_save_dir, j);

        % create .csv file
        write_fetch_scene_to_csv(W, filename);

    end
end