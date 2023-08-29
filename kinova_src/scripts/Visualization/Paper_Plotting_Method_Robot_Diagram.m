close all;
clear all;
clc;

%% Setup Robot

% load robot
robot_name = 'Kinova_Grasp_w_Tray_Gripper.urdf';
robot = importrobot(robot_name);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
% get parameters
add_uncertainty_to = 'all'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

params = load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);

% controller info
LLC_info.ultimate_bound = 0.00191; % 0.00191
LLC_info.Kr = 10;

% create link poly zonotopes
% [link_poly_zonotopes, link_sizes, meshes] = create_link_poly_zonos(robot);
% zono_order = 10;

% contact parameters
u_s = 0.5;
surf_rad = 0.05; % meter

% compute trajectories
% initial conditions
q_0 = [0;-pi/4;0;-pi/2;0;pi/4;0];
qd_0= [-pi/2;pi/30;0;0;0;0;0];
qdd_0 = [0;pi/30;0;0;0;0;0];

% create pz trajectories
joint_axes = [zeros(2, length(q_0)); ones(1, length(q_0))]; % Question: what is this?
taylor_degree = 5;
traj_type = 'bernstein';
add_ultimate_bound = true;

% create JRS
[Q_des, Qd_des, Qdd_des, Q, Qd, Qd_a, Qdd_a, R_des, R_t_des, R, R_t, T, E_p, jrs_info] = create_jrs_online_modified(q_0,qd_0,qdd_0, joint_axes, taylor_degree, traj_type, add_ultimate_bound, LLC_info);

% trajectory parameter
kvec = [0; 0.5; 0.0; 0.5; 0.0; 0.5; 0];

% planning info
t_traj = 0:jrs_info.dt:jrs_info.t_f;
t_traj(end) = [];

% get JRS info
id_tmp = jrs_info.id;
id_names_tmp = jrs_info.id_names;
jrs_info.id = {};
jrs_info.id_names = {};
for i=1:length(kvec)
    jrs_info.id{i,1} = id_tmp(i,:);
    jrs_info.id_names{i,1} = id_names_tmp(i,:);
end

% create planner
P.jrs_info = jrs_info;
P.t_start = 0;
P.t_stop = 1;
P.t_plan = 0.5;
P.traj_type = traj_type;

% compute desired trajectory
t_steps_dt = 0:jrs_info.dt:1;
t_steps = 0:jrs_info.dt:1;
n_steps = length(t_steps_dt);
q_des = zeros(length(kvec), length(t_steps));
q_des_dt = zeros(length(kvec), n_steps);
qd_des = zeros(length(kvec), n_steps);
qdd_des = zeros(length(kvec), n_steps);
for i = 1:length(t_steps)
    [q_des(:,i), qd_des(:,i), qdd_des(:,i)] = desired_trajectory(P, q_0, qd_0, qdd_0, t_steps(i), kvec);
end
for i = 1:jrs_info.n_t
    [q_des_dt(:,i), ~, ~] = desired_trajectory(P, q_0, qd_0, qdd_0, t_steps_dt(i), kvec);
end

%% Poses to Plot

test = [];

for i = 1:length(q_des)
    robot_poses{i} = copy(robot);
end


%% Plotting Poses

close all;
figure(202)

rbtpatches = [];
% plot robot in start pose
ax1 = show(robot,q_des(:,1),'Frames','off',PreservePlot=false,FastUpdate=true);
% find patches of first robot to keep non-transparent
mesh_names = robot.BodyNames;
for i = 1:length(mesh_names)
    rbtpatches = [rbtpatches findobj(ax1.Children,'Type','patch','-regexp','DisplayName',mesh_names{i})];
    for j = 1:size(rbtpatches,2)
        set(rbtpatches(:,j),'FaceAlpha',1);
    end
end
hold on

% select time instances to plot
idx_to_plot = [7 10 15 21];

% plot transparent poses
for i = 1:length(idx_to_plot)
    % plot pose
    ax{idx_to_plot(i)} = show(robot_poses{idx_to_plot(i)},q_des(:,idx_to_plot(i)),'Frames','off',PreservePlot=false,FastUpdate=true);

    for j = 1:length(mesh_names)
        rbtpatchesnew=findobj(ax{idx_to_plot(i)}.Children,'Type','patch','-regexp','DisplayName',mesh_names{j});
        copyrbtpatches=rbtpatchesnew(~ismember(rbtpatchesnew,rbtpatches));
        if i == length(idx_to_plot)
            set(copyrbtpatches,'FaceAlpha',0.5);
            set(copyrbtpatches,'FaceColor',[144 238 144]./255)
        else
            set(copyrbtpatches,'FaceAlpha',0.3);
        end
        rbtpatches = [rbtpatches copyrbtpatches]; % rbtpatchesnew(2*i:end,1)];
    end
    
end


% plot formatting
view(-110,15)
xlim([-1.5 0.15])
ylim([-0.75 0.75])
zlim([0 1])
grid off
material dull
% lightangle(0,20)

%% Helper Functions

function [q_des, qd_des, qdd_des] = desired_trajectory(P, q_0, q_dot_0, q_ddot_0, t, k)
    % at a given time t and traj. param k value, return
    % the desired position, velocity, and acceleration.
    switch P.traj_type
    case 'orig'
        t_plan = P.t_plan;
        t_stop = P.t_stop;
        k_scaled = P.jrs_info.c_k_orig + P.jrs_info.g_k_orig.*k;
        
        if ~isnan(k)
            if t <= t_plan
                % compute first half of trajectory
                q_des = q_0 + q_dot_0*t + (1/2)*k_scaled*t^2;
                qd_des = q_dot_0 + k_scaled*t;
                qdd_des = k_scaled;
            else
                % compute trajectory at peak
                q_peak = q_0 + q_dot_0*t_plan + (1/2)*k_scaled*t_plan^2;
                qd_peak = q_dot_0 + k_scaled*t_plan;

                % compute second half of trajectory
                q_des = q_peak + qd_peak*(t-t_plan) + (1/2)*((0 - qd_peak)/(t_stop - t_plan))*(t-t_plan)^2;
                qd_des = qd_peak + ((0 - qd_peak)/(t_stop - t_plan))*(t-t_plan);
                qdd_des = (0 - qd_peak)/(t_stop - t_plan);
            end
        else
            % bring the trajectory to a stop in t_plan seconds
            % trajectory peaks at q_0
            q_peak = q_0;
            qd_peak = q_dot_0;
            
            if t <= t_plan && ~all(q_dot_0 == 0) % we're braking!
                q_des = q_peak + qd_peak*t + (1/2)*((0 - qd_peak)/t_plan)*t^2;
                qd_des = qd_peak + ((0 - qd_peak)/t_plan)*t;
                qdd_des = (0 - qd_peak)/t_plan;
            else % we should already be stopped, maintain that.
                q_des = q_peak;
                qd_des = zeros(size(q_dot_0));
                qdd_des = zeros(size(q_0));
            end
        end
    case 'bernstein'
        % assuming K = [-1, 1] corresponds to final position for now!!
        n_q = length(q_0);
        if ~isnan(k)
            q1 = q_0 + P.jrs_info.c_k_bernstein + P.jrs_info.g_k_bernstein.*k;
            for j = 1:n_q
                beta{j} = match_deg5_bernstein_coefficients({q_0(j); q_dot_0(j); q_ddot_0(j); q1(j); 0; 0});
                alpha{j} = bernstein_to_poly(beta{j}, 5);
            end
            q_des = zeros(length(q_0), 1);
            qd_des = zeros(length(q_0), 1);
            qdd_des = zeros(length(q_0), 1);
            for j = 1:n_q
                for coeff_idx = 0:5
                    q_des(j) = q_des(j) + alpha{j}{coeff_idx+1}*t^coeff_idx;
                    if coeff_idx > 0
                        qd_des(j) = qd_des(j) + coeff_idx*alpha{j}{coeff_idx+1}*t^(coeff_idx-1);
                    end
                    if coeff_idx > 1
                        qdd_des(j) = qdd_des(j) + (coeff_idx)*(coeff_idx-1)*alpha{j}{coeff_idx+1}*t^(coeff_idx-2);
                    end
                end
            end
        else
            % bring the trajectory to a stop using previous trajectory...
            t_plan = P.t_plan;
            if t <= t_plan && norm(q_dot_0) > 1e-8 && norm(q_dot_0) > 1e-8
                % just plug into previous trajectory, but shift time forward by t_plan.
                [q_des, qd_des, qdd_des] = P.info.desired_trajectory{end - 1}(t + t_plan);
            else % we should already be stopped, maintain that.
                q_des = q_0;
                qd_des = zeros(n_q, 1);
                qdd_des = zeros(n_q, 1);
            end
        end
    otherwise
        error('trajectory type not recognized');
    end
end