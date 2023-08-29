%% RTD-Force Matlab and Realtime Constraint Comparison
% Zachary Brei
% 01/13/2023

clear all; close all; clc;

%% Robot Parameters
agent_urdf = 'Kinova_Grasp_URDF.urdf';

% Kinova Ford Demo Values
u_s = 0.609382421; % static coefficient of friction
db2 = 0.058; % 58; % (m) diameter of circular contact area (inscribed in square base of motion tracking cube)

input_constraints_flag = false;
grasp_constraints_flag = true;

add_uncertainty_to = 'all'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);
use_gravity = true;

for i=1:7
    % lower limit
    joint_position_limits(1,i) = robot.Bodies{1, i}.Joint.PositionLimits(1);
    % upper limit
    joint_position_limits(2,i) = robot.Bodies{1, i}.Joint.PositionLimits(2);
end

joint_speed_limits = [-1.3963, -1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218;
                       1.3963,  1.3963,  1.3963,  1.3963,  1.2218,  1.2218,  1.2218]; % matlab doesn't import these from urdf so hard code into class
joint_input_limits = [-56.7, -56.7, -56.7, -56.7, -29.4, -29.4, -29.4;
                       56.7,  56.7,  56.7,  56.7,  29.4,  29.4,  29.4]; % matlab doesn't import these from urdf so hard code into class
transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class
M_min_eigenvalue = 5.095620491878957; % matlab doesn't import these from urdf so hard code into class

add_measurement_noise_ = false;
measurement_noise_size_ = [];

max_travel = 0.4;

% obstacles = {};
obstacles{1} = box_obstacle_zonotope('center', [3; 0; 0.6],...
                                     'side_lengths', [0.1; 0.1; 0.1]) ;

goal_type = 'configuration'; % pick 'end_effector_location' or 'configuration'
goal_radius = pi/30;
dimension = 3 ;
verbosity = 10;
use_robust_input = true;
LLC_V_max = 5e-5;
agent_move_mode = 'integrator' ; % pick 'direct' or 'integrator'
use_CAD_flag = true;
specify_color_flag = false;
link_color_data = {[],[],[],[],[],[],[],[],[169 169 169]./255,[255 22 12]./255};
% link_FaceAlpha = {[],[],[],[],[],[],[],[],[],[0]};

%% Create list of random conditions to test

num_conditions = 1;

% for i = 1:num_conditions
%     % r = a + (b-a).*rand(N,1)
%     for j = 1:7
%         
%         if joint_position_limits(2,j) > 100
%             % random start positions
%             q_0(i,j) = rand();
%             % random goal positions
%             q_goal(i,j) = rand();
%         else
%             % random start positions
%             q_0(i,j) = joint_position_limits(1,j) + (joint_position_limits(2,j)-joint_position_limits(1,j))*rand();
%             % random goal positions
%             q_goal(i,j) = joint_position_limits(1,j) + (joint_position_limits(2,j)-joint_position_limits(1,j))*rand();
%         end
%         
%         % generate waypoint for desired position
%         dir = q_goal(i,j)-q_0(i,j);
%         dir = dir./norm(dir);
%         q_des(i,j) = q_0(i,j)+max_travel*dir;
%         % random start velocity
%         q_dot_0(i,j) = joint_speed_limits(1,j) + (joint_speed_limits(2,j)-joint_speed_limits(1,j))*rand(1,1);
%         % random start acceleration (what interval?)
%         q_ddot_0(i,j) = -1 + (1+1)*rand(1,1);
%     end
% end

% debugging: fail, success, fail
% q_0 = [0,pi/2,0,0,0,0,0;0,-pi/2,0,0,0,0,0;0,pi/2,0,0,0,0,0];
% q_dot_0 = zeros(3,7);
% q_ddot_0 = zeros(3,7);
% q_des = zeros(3,7);

% debugging: success
q_0 = [0,-pi/2,0,0,0,0,0];
q_dot_0 = zeros(1,7);
q_ddot_0 = zeros(1,7);
q_des = zeros(1,7);

%%

W = kinova_world_static('create_random_obstacles_flag', false, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
                        'verbose',verbosity, 'start', q_0, 'goal', q_des, 'obstacles', obstacles, 'goal_type', goal_type) ;

% agent
A = uarmtd_agent(robot, params,...
                 'verbose', verbosity,...
                 'animation_set_axes_flag', 0,... 
                 'animation_set_view_flag', 0,...
                 'move_mode', agent_move_mode,...
                 'use_CAD_flag', use_CAD_flag,...
                 'joint_speed_limits', joint_speed_limits, ...
                 'joint_input_limits', joint_input_limits, ...
                 'add_measurement_noise_', add_measurement_noise_, ...
                 'measurement_noise_size_', measurement_noise_size_, ...
                 'M_min_eigenvalue', M_min_eigenvalue, ...
                 'animation_playback_rate',1, ...
                 'animation_time_discretization',0.05);
%                  'u_s', u_s, ...
%                  'db2', db2);
                %'specify_color_flag',specify_color_flag,...
                 %'link_color_data',link_color_data,...

% LLC
if use_robust_input
    A.LLC = uarmtd_robust_CBF_LLC('verbose', verbosity, ...
                                  'use_true_params_for_robust', false, ...
                                  'V_max', LLC_V_max, ...
                                  'if_use_mex_controller', false);
else
    A.LLC = uarmtd_nominal_passivity_LLC('verbose', verbosity);
end

A.LLC.setup(A);

P = uarmtd_planner('verbose', verbosity, ...
                   'first_iter_pause_flag', true, ...
                   'use_q_plan_for_cost', true, ...
                   'input_constraints_flag', input_constraints_flag, ...
                   'grasp_constraints_flag', grasp_constraints_flag,...
                   'use_robust_input', use_robust_input, ...
                   'traj_type', 'bernstein', ...
                   'use_cuda', false,...
                   'use_waypoint_for_bernstein_center',true,...
                   'u_s', u_s, ...
                   'db2', db2);

agent_info = A.get_agent_info() ;
agent_info.LLC_info = A.LLC.get_LLC_info();
world_info = W.get_world_info(agent_info,P) ;
P.setup(agent_info,world_info) ;
P.agent_info = agent_info;
% W = kinova_world_static('create_random_obstacles_flag', false, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
%                         'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles, 'goal_type', goal_type) ;


%% C++ Method

%% Calling Realtime Planner

% !!!!!!
% needs to match c++ code
% !!!!!!
% P.jrs_info.n_t = 128;
% P.jrs_info.n_q = 7;
% P.jrs_info.n_k = 7;
% P.jrs_info.c_k_bernstein = zeros(7,1);

% !!!!!!
% Make sure this is consistent with the k_range in
% cuda-dev/PZsparse-Bernstein/Trajectory.h 
% !!!!!!
% P.jrs_info.g_k_bernstein = [pi/72; pi/72; pi/72; pi/72; pi/72; pi/72; pi/72];

for idx_real = 1:num_conditions

    % organize input to cuda program
    fprintf('Calling CUDA & C++ Program!')
    cuda_input_file = fopen('/home/roahmlab/Documents/armour-dev/kinova_src/kinova_simulator_interfaces/kinova_planner_realtime/buffer/armour.in', 'w');
    
    for ind = 1:size(q_0,2)
        fprintf(cuda_input_file, '%.10f ', q_0(idx_real,ind));
    end
    fprintf(cuda_input_file, '\n');
    for ind = 1:size(q_dot_0,2)
        fprintf(cuda_input_file, '%.10f ', q_dot_0(idx_real,ind));
    end
    fprintf(cuda_input_file, '\n');
    for ind = 1:size(q_ddot_0,2)
        fprintf(cuda_input_file, '%.10f ', q_ddot_0(idx_real,ind));
    end
    fprintf(cuda_input_file, '\n');
    for ind = 1:size(q_des,2)
        fprintf(cuda_input_file, '%.10f ', q_des(idx_real,ind));
    end
    fprintf(cuda_input_file, '\n');
    fprintf(cuda_input_file, '%d\n', max(length(obstacles), 0));
    for obs_ind = 1:length(obstacles)
        temp = reshape(obstacles{obs_ind}.Z, [1,size(obstacles{obs_ind}.Z,1) * size(obstacles{obs_ind}.Z,2)]); %world_info.
        for ind = 1:length(temp)
            fprintf(cuda_input_file, '%.10f ', temp(ind));
        end
        fprintf(cuda_input_file, '\n');
    end
    
    fclose(cuda_input_file);
    
    % call cuda program in terminal
    % you have to be in the proper path!
    %                     terminal_output = system('./../kinova_simulator_interfaces/kinova_planner_realtime/armour_main'); % armour path
    terminal_output = system('/home/roahmlab/Documents/armour-dev/kinova_src/kinova_simulator_interfaces/kinova_planner_realtime/rtd_force_main_v2'); % rtd-force path
    
    % To do, figure out how to read and store output for comparison
    
    if terminal_output == 0
        data = readmatrix('armour.out', 'FileType', 'text');
        k_opt = data(1:end-1);
        planning_time(idx_real) = data(end) / 1000.0; % original data is milliseconds
    
        if length(k_opt) == 1
            fprintf('Unable to find new trajectory!')
            k_opt = nan;
        else
            fprintf('New trajectory found!');
            for i = 1:length(k_opt)
                fprintf('%7.6f ', k_opt);
            end
            fprintf('\n');
        end
    else
        error('CUDA program error! Check the executable path in armour-dev/kinova_src/kinova_simulator_interfaces/uarmtd_planner');
    end
    
    if terminal_output == 0
        % read FRS information if needed
        joint_frs_center{idx_real} = readmatrix('armour_joint_position_center.out', 'FileType', 'text');
        joint_frs_radius{idx_real} = readmatrix('armour_joint_position_radius.out', 'FileType', 'text');
        control_input_radius(:,idx_real) = readmatrix('armour_control_input_radius.out', 'FileType', 'text');
        constraints_value(:,idx_real) = readmatrix('armour_constraints.out', 'FileType', 'text');
    else
        k_opt = nan;
    end
    for kk = 1:length(k_opt)
        k_opt_storage(kk,idx_real) = k_opt(kk);
    end
    fprintf('\n \n')
end


%% Matlab Method


% generate constraints
% note that the if statements in this function in the uarmtd_planner.m file need to be commented out to
% save all of the constraints for comparison.

for i=1:num_conditions

%     matlab_constraints{i} = generate_constraints(P, q_0(i,:), q_dot_0(i,:), q_ddot_0(i,:), obstacles, q_des(i,:));

    % Ground Truth

    [u{i}, f{i}, n{i}] = rnea(q_0(i,:), q_dot_0(i,:), q_dot_0(i,:), q_ddot_0(i,:), use_gravity, params.nominal)
end


%% Comparison
clear ml_slip ml_tip ml_sep ml_constraints ml_constraints_sliced sep_error slip_error tip_error max_slip_error max_tip_error max_sep_error

counter = 1;

for i = 1:num_conditions
    % Real-time: 
    % pull out constraints values
    rt_sep = constraints_value(701:800,i);
    rt_slip = constraints_value(801:900,i);
    rt_tip = constraints_value(901:1000,i);

    % Matlab:
    % pull out constraint values
    ml_constraints = matlab_constraints{i}.constraints;
    % slice with respect to the k found by the real-time
    for j = 1:length(ml_constraints)
        ml_constraints_sliced{j,i} = ml_constraints{j}(k_opt_storage(:,i))';
    end

    % replace uarmtd_planner.m constraint functions with zonotope_slice()
    % instead of slice()?

    ml_constraints_sliced = cell2mat(ml_constraints_sliced);
    ml_sep = ml_constraints_sliced(1:3:end);
    ml_slip = ml_constraints_sliced(2:3:end);
    ml_tip = ml_constraints_sliced(3:3:end);
%     for j = 1:length(ml_constraints_sliced)
%         if counter == 1
%             ml_sep(j,i) = ml_constraints_sliced(j,i);
%             counter = counter + 1;
%         end
%         if counter == 2
%             ml_slip(j,i) = ml_constraints_sliced(j,i);
%             counter = counter + 1;
%         end
%         if counter == 3
%             ml_tip(j,i) = ml_constraints_sliced(j,i);
%             counter = counter + 1;
%         end
%         if counter == 4
%             counter = 1;
%         end
%     end
    
    % Calculate difference
    sep_error = (rt_sep - ml_sep)./ml_sep;
    slip_error = (rt_slip - ml_slip)./ml_slip;
    tip_error = (rt_tip - ml_tip)./ml_tip;

    max_sep_error = max(sep_error)
    max_slip_error = max(slip_error)
    max_tip_error = max(tip_error)

    %% Check Ground Truth Exist In the Overapproximations

    gt_sep = -1*f{i}(3,10);
    

end
