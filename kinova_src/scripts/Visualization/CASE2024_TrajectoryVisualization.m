%% CASE 2024 Trajectory Visualization
% Zachary Brei

clear all;
close all;
clc;
fig_num = 0;

%% Parameters

% Initial state
q_0 = [pi/4;-pi/4;0;-pi/2;0;pi/4;0];
qd_0= [-pi/4;pi/30;0;0;0;0;0];
qdd_0 = [0;pi/30;0;0;0;0;0];

% Trajectory parameter
kvec = [0.001; 0.48; 0.001; 0.48; 0.001; 0.48; 0.001];

% The center should be the initial position?
P.bernstein_center = zeros(size(q_0));
P.bernstein_final_range = [pi/24; pi/72; pi/24; pi/72; pi/72; pi/72; pi/72]; % added critical abs() here!!

%% Create Desired Trajectory

t_cont = linspace(0,1); % 1/40 for a single iteration

for i = 1:length(t_cont)
    [q_cont_des(:,i), qd_cont_des(:,i), qdd_cont_des(:,i)] = desired_trajectory(P, q_0, qd_0, qdd_0, t_cont(i), kvec);
end

%% Create Overapproximation of Desired Trajectory

%% Plotting

fig_num = fig_num + 1;
fig = figure(fig_num);
% subplot(3,1,1)
plot(t_cont,rad2deg(q_cont_des(1,:)),'LineWidth',2)
% subplot(3,1,2)
% plot(t_cont,rad2deg(qd_cont_des(1,:)))
% subplot(3,1,3)
% plot(t_cont,rad2deg(qdd_cont_des(1,:)))
xlabel('Time (s)')
ylabel('Joint Angle (deg)')
set(gcf,'Color','w');
fontsize(fig, 14, "points")

%% Extra

% % Forming overapproximated trajectory
% for j = 1:n_q
%    q1{j, 1} = q(j) + bernstein_center(j) + bernstein_final_range(j).*K{j}; % final position is initial position +- k \in [-1, 1]
%    dq1 = 0;
%    ddq1 = 0;
%    beta{j} = match_deg5_bernstein_coefficients({q(j); dq(j); ddq(j); q1{j}; dq1; ddq1});
%    alpha{j} = bernstein_to_poly(beta{j}, 5);
% end
% 
% % main loop:
% for i = 1:n_t
% 	for j = 1:n_q
%         Q_des{i}{j, 1} = 0;
%         Qd_des{i}{j, 1} = 0;
%         Qdd_des{i}{j, 1} = 0;
%         for k = 0:5
%             Q_des{i}{j, 1} = Q_des{i}{j, 1} + alpha{j}{k+1}.*T{i}.^k;
%             if k > 0
%                 Qd_des{i}{j, 1} = Qd_des{i}{j, 1} + k*alpha{j}{k+1}.*T{i}.^(k-1);
%             end
%             if k > 1
%                 Qdd_des{i}{j, 1} = Qdd_des{i}{j, 1} + (k)*(k-1)*alpha{j}{k+1}.*T{i}.^(k-2);
%             end
%         end
% 
% 		% add tracking error
% 		Q{i}{j, 1} = Q_des{i}{j, 1} + E_p{j};
% 		Qd{i}{j, 1} = Qd_des{i}{j, 1} + E_v{j};
% 		Qd_a{i}{j, 1} = Qd_des{i}{j, 1} + k_r.*E_p{j};
% 		Qdd_a{i}{j, 1} = Qdd_des{i}{j, 1} + k_r.*E_v{j};
%         
% 		% get rotation matrices:
% 		[R_des{i}{j, 1}, R_t_des{i}{j, 1}] = get_pz_rotations_from_q(Q_des{i}{j, 1}, joint_axes(:, j), taylor_degree);
% 		[R{i}{j, 1}, R_t{i}{j, 1}] = get_pz_rotations_from_q(Q{i}{j, 1}, joint_axes(:, j), taylor_degree);
% 	end
% end

%% Helper Functions

function [q_des, qd_des, qdd_des] = desired_trajectory(P, q_0, q_dot_0, q_ddot_0, t, k)
    % at a given time t and traj. param k value, return
    % the desired position, velocity, and acceleration.
    
    % assuming K = [-1, 1] corresponds to final position for now!!
    n_q = length(q_0);
    if ~isnan(k)
        q1 = q_0 + P.bernstein_center + P.bernstein_final_range.*k;
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
end