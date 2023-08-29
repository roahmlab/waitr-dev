% 0. provide proper input data in buffer/armour.in
% 1. run ./compile_debug_script.sh
% 2. run ./test

close all; clear; clc;

fig_num = 0;

%% initialize robot
robot = importrobot('Kinova_Grasp_w_Tray.urdf');
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot, ...
                           'add_uncertainty_to', 'all', ...
                           'uncertain_mass_range', [0.97, 1.03]);

A = uarmtd_agent(robot, params,...
                 'animation_set_axes_flag', 0,... 
                 'animation_set_view_flag', 0,...
                 'use_CAD_flag', true,...
                 'add_measurement_noise_', false, ...
                 'measurement_noise_size_', 0);
transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class

link_poly_zonotopes = create_pz_bounding_boxes(robot);

%% initialize desired trajectories
% choose random initial conditions and make sure they are aligned with
% the first three rows in buffer/armour.in
% basic start
q0 = [-2.53318530717959	-1.04720000000000	0.0	-2.09440000000000	0.0	1.57080000000000	0.0 ]';
qd0 = [0.0000000000 -0.00000000000 0.0000000000 0.0000000000 0.0000000000 0.0000000000 0.0000000000 ]';
qdd0 = [0.0000000000 0.00000000000 0.0000000000 0.0000000000 0.0000000000 0.0000000000 0.0000000000 ]';
qdes = [0.0000000000 -1.5707963268 0.0000000000 0.0000000000 0.0000000000 0.0000000000 0.1000000000  ]';
% random
% q0 = [1.8050949857 1.5523818073 -0.5593660299 0.1468763715 -0.5267268712 -0.2220512642 -2.0707212135  ]';
% qd0 = [0.0075302418 0.0092257990 -0.0049952047 -0.0036228836 -0.0040500586 -0.0016993588 -0.0063685653  ]';
% qdd0 = [0.0009060653 -0.0046286801 0.0211882473 -0.0066294695 0.0218570425 -0.0048845454 0.0208029198 ]';
% qdes = [-2.6499539903 0.8150094930 0.9830580739 -2.4237010208 -0.4592416536 0.5571133216 1.0683250709 ]';

% hardware start
% -2.53318530717959	-1.04720000000000	0.0	-2.09440000000000	0.0   1.57080000000000	0.0

% choose a random k_range and make sure they are aligned with k_range in
% Parameters.h
% k_range = [pi/24, pi/24, pi/24, pi/24, pi/24, pi/24, pi/24]';
% k_range = [pi/36, pi/36, pi/36, pi/36, pi/36, pi/36, pi/36]';
k_range = [pi/48, pi/48, pi/48, pi/48, pi/48, pi/48, pi/48]';
% k_range = [pi/72, pi/72, pi/72, pi/72, pi/72, pi/72, pi/72]';


% choose a random point to slice and make sure they are equal to variable
% factors defined in PZ_test.cpp

% k = [0.5, 0.7, 0.7, 0.0, -0.8, -0.6, -0.7]'; % * 0;
% k = zeros(7,1);
k = ones(7,1);
% k = -ones(7,1);
% k=[0.999901, -0.898396, 0.973896, -0.454472, 0.999083, 0.940592, 0.974109]';

% desired trajectory constraints
q1 = q0 + k .* k_range; % final position is k*k_range away from initial
qd1 = zeros(7,1); % final velocity is zero
qdd1 = zeros(7,1); % final acceleration is zero

% for tid = 1:128
duration = 1.75;
tid = 50;
tspan = linspace(0, duration, tid + 1);

duration_onesec = 1;
tspan_onesec = linspace(0, duration_onesec, tid+1);

beta = match_deg5_bernstein_coefficients({q0, qd0, qdd0, q1, qd1, qdd1}, duration);
beta_onesec = match_deg5_bernstein_coefficients({q0, qd0, qdd0, q1, qd1, qdd1}, duration_onesec);

%% read CUDA output

link_reachset_center = readmatrix('buffer/armour_joint_position_center.out', 'FileType', 'text');
link_reachset_generators = readmatrix('buffer/armour_joint_position_radius.out', 'FileType', 'text');

torque_reachset_center = readmatrix('buffer/armour_constraints.out', 'FileType', 'text');
torque_reachset_radius = readmatrix('buffer/armour_control_input_radius.out', 'FileType', 'text');

force_reachset_values = readmatrix('buffer/armour_wrench_values.out', 'FileType', 'text');
force_constraint_values = readmatrix('buffer/armour_force_constraint_radius.out', 'FileType', 'text');

des_traj_slice = readmatrix('buffer/armour_desired_sliced.out', 'FileType', 'text');

% %% Processing CUDA output
% 
% % separate the desired trajectories
% des_vel_center = des_traj_slice(1:100,1:2:14);
% des_vel_radius = des_traj_slice(1:100,2:2:14);
% des_aux_vel_center = des_traj_slice(101:200,1:2:14);
% des_aux_vel_radius = des_traj_slice(101:200,2:2:14);
% des_accel_center = des_traj_slice(201:300,1:2:14);
% des_accel_radius = des_traj_slice(201:300,2:2:14);
% 
% % separate the force arrays
% f_rs_c = force_reachset_values(:,1:3);
% n_rs_c = force_reachset_values(:,4:6);
% f_rs_r = force_reachset_values(:,7:9);
% n_rs_r = force_reachset_values(:,10:12);
% sep_ub_cuda = force_constraint_values(1:100,1);
% slip_ub_cuda = force_constraint_values(101:200,1);
% tip_ub_cuda = force_constraint_values(201:300,1);
% sep_lb_cuda = force_constraint_values(1:100,2);
% slip_lb_cuda = force_constraint_values(101:200,2);
% tip_lb_cuda = force_constraint_values(201:300,2);
% 
% 
% 
% 
% %% Calculating Nominal Values
% 
% us = zeros(7,tid);
% fs = cell(1,tid);
% ns = cell(1,tid);
% ts = zeros(1,tid);
% for i = 1:tid
%     % choose a random time inside this time interval
%     t_lb = tspan(i);
%     t_ub = tspan(i + 1);
%     temp = rand;
%     ts(i) = (t_ub - t_lb) * temp + t_lb;
% 
%     t_lb_onesec = tspan_onesec(i);
%     t_ub_onesec = tspan_onesec(i+1);
%     ts_onesec(i) = (t_ub_onesec - t_lb_onesec) * temp + t_lb_onesec;
% 
%     [q, qd, qdd] = get_desired_traj(beta, ts(i), duration);
%     [q_onesec, qd_onesec, qdd_onesec] = get_desired_traj(beta_onesec, ts_onesec(i), duration_onesec);
%     
%     q_des_matlab(:,i) = q;
%     qd_des_matlab(:,i) = qd;
%     qdd_des_matlab(:,i) = qdd;
% 
%     q_des_matlab_onesec(:,i) = q_onesec;
%     qd_des_matlab_onesec(:,i) = qd_onesec;
%     qdd_des_matlab_onesec(:,i) = qdd_onesec;
% 
%     [us(:,i), fs{i}, ns{i}] = rnea(q, qd, qd, qdd, true, params.nominal); % + transmision_inertia' .* qdd;
% end
% 
% %% Plotting Desired Trajectory Comparison
% 
% % Trajectory Duration Verification
% % plotting one second trajecotry vs duration trajectory
% fig_num = fig_num + 1;
% figure(fig_num);
% hold on;
% 
% for i = 1:7
%     % plotting position
%     subplot(7,3,3*i-2)
%     hold on
%     plot(ts, q_des_matlab(i,:))
%     plot(ts_onesec,q_des_matlab_onesec(i,:))
%     % plotting velocity
%     subplot(7,3,3*i-1)
%     hold on
%     plot(ts, qd_des_matlab(i,:))
%     plot(ts_onesec,qd_des_matlab_onesec(i,:))
%     % plotting acceleration
%     subplot(7,3,3*i)
%     hold on
%     plot(ts, qdd_des_matlab(i,:))
%     plot(ts_onesec,qdd_des_matlab_onesec(i,:))
% end
% sgtitle('Trajectory Duration Comparison')
% 
% % Outputting check if the end position states are the same!!!
% position_end_check = (q_des_matlab(:,end) - q_des_matlab_onesec(:,end) < 1e-10)
% % velocity and acceleration end should be zero
% % should initial all be the same?
% 
% fig_num = fig_num + 1;
% figure(fig_num);
% hold on;
% for i = 1:7
%     subplot(7,1,i)
%     hold on
%     plot(ts,qd_des_matlab(i,:),'ok')
% %     plot(ts,des_vel_center(:,i),'-*r')
%     plot(ts,des_aux_vel_center(:,i),'--b')
%     % plot(ts,des_vel_center+des_vel_radius,'--r')
%     % plot(ts,des_vel_center-des_vel_radius,'--r')
% end
% sgtitle('Desired Velocity Comparison')
% 
% fig_num = fig_num + 1;
% figure(fig_num); 
% hold on;
% 
% for i = 1:7
%     subplot(7,1,i)
%     hold on
%     plot(ts,qdd_des_matlab(i,:),'ok')
%     plot(ts,des_accel_center(:,i),'--b')
%     % plot(ts,des_accel_center+des_vel_radius,'--r')
%     % plot(ts,des_accel_center-des_vel_radius,'--r')
% end
% sgtitle('Desired Acceleration Comparison')
% 
% % these won't match perfectly because c++ center is at the center of each
% % time interval and the matlab value is at a random point in the time
% % interval.
% vel_check = qd_des_matlab(1,:)' ./ des_vel_center(:,1);
% accel_check = qdd_des_matlab(1,:)' ./ des_accel_center(:,1);
% 
% %% Plotting Torque Reach Sets
% 
% % u_lb = torque_reachset_center - torque_reachset_radius;
% % u_ub = torque_reachset_center + torque_reachset_radius;
% % 
% % figure(2)
% % % there is a better way to do this
% % for i = 1:7
% %     subplot(3,3,i);
% %     hold on;
% %     plot(ts, us(i,:), 'r');
% %     plot(ts, u_lb(:,i), 'b');
% %     plot(ts, u_ub(:,i), 'b');
% %     title(['link ', num2str(i)]);
% %     xlabel('time (sec)');
% %     ylabel('torque (N*m)');
% % end
% % sgtitle('sliced torque reachable set');
% 
% %% Plotting Force and Moment Reach Sets
% 
% % extract the nominal values from the cells
% for j = 1:tid
%     f_nom(j,:) = fs{j}(:,10)';
%     n_nom(j,:) = ns{j}(:,10)';
% end
% 
% % get the upper and lower bounds of the force reach sets
% f_ub = f_rs_c + f_rs_r;
% f_lb = f_rs_c - f_rs_r;
% n_ub = n_rs_c + n_rs_r;
% n_lb = n_rs_c - n_rs_r;
% 
% fig_num = fig_num + 1;
% figure(fig_num); 
% hold on;
% 
% plot_label = {'X-axis','Y-axis','Z-axis'};
% for i = 1:3
%     subplot(3,2,i*2-1)
%     hold on
%     plot(ts,f_nom(:,i),'ok')
%     plot(ts,f_ub(:,i),'-b')
%     plot(ts,f_lb(:,i),'-b')
%     title([plot_label(i),' Force'])
%     xlabel('Time (sec)')
%     ylabel('Force (Newton)')
% end
% for i = 1:3
%     subplot(3,2,i*2)
%     hold on
%     plot(ts,n_nom(:,i),'ok')
%     plot(ts,n_ub(:,i),'-b')
%     plot(ts,n_lb(:,i),'-b')
%     title([plot_label(i),' Moment'])
%     xlabel('Time (sec)')
%     ylabel('Moment (Newton*meter)')
% end
% 
% %% Calculate the Constraints
% 
% u_s = 0.609382421;
% surf_rad =  0.058/2;
% 
% for i = 1:tid
%     % separation constraint
%     con_mat(i,1) = -1*f_nom(i,3);
%     % slip constraint
%     con_mat(i,2) = f_nom(i,1)^2 + f_nom(i,2)^2 - u_s^2*f_nom(i,3)^2;
%     % tip constraint
%     ZMP_top = cross([0;0;1],n_nom(i,:));
%     ZMP_bottom = dot([0;0;1],f_nom(i,:));
%     con_mat(i,3) = ZMP_top(1)^2+ZMP_top(2)^2 - surf_rad^2*ZMP_bottom^2;
% end
% 
% fig_num = fig_num + 1;
% figure(fig_num); 
% hold on;
% 
% constraint_label = {'Separation Constraint','Slipping Constraint','Tipping Constraint'};
% for i = 1:3
%     subplot(3,1,i)
%     hold on
%     plot(ts,con_mat(:,i),'-k')
%     plot(ts,force_constraint_values((1+(i-1)*100):(100+(i-1)*100),1),'b-')
%     plot(ts,force_constraint_values((1+(i-1)*100):(100+(i-1)*100),2),'b-')
%     title(constraint_label(i))
%     xlabel('Time (sec)')
%     ylabel('Constraint Value')
% end
% 
% %% Plotting the Force Constraints

% figure(4)
% for i=1:3
% 
% end

% for i = 1:length(A.time)
% 
%     out = W.grasp_check(A,A.agent_info,P.info)
% 
% end

%% Plotting Link Reach Sets

tid = 50;

% fig_num = fig_num + 1;
% figure(fig_num); 
% hold on; view(3); axis equal; axis on;

% choose a random time inside this time interval
t_lb = tspan(tid);
t_ub = tspan(tid + 1);
t = (t_ub - t_lb) * rand + t_lb;

q_rand = get_desired_traj(beta, t, duration);

fig_num = fig_num + 1;
figure(fig_num);
hold on;
% plot robot
A.plot_at_time(q_rand);
view(3)
axis('equal')
xlim([-1.5 1.5])
ylim([-1.5 1.5])
zlim([0 1.5])
grid on;

% plot link reachsets
numBodies = 8;
for j = 1:numBodies
    c = link_reachset_center((tid-1)*numBodies+j, :)';
    g = link_reachset_generators( ((tid-1)*numBodies+j-1)*3+1 : ((tid-1)*numBodies+j)*3, :);
    Z = zonotope(c, g);
    Z_v = vertices(Z)';
    trisurf(convhulln(Z_v),Z_v(:,1),Z_v(:,2),Z_v(:,3),'FaceColor',[0,0,1],'FaceAlpha',0.1,'EdgeColor',[0,0,1],'EdgeAlpha',0.3);
end
% lighting flat
% end

%% helper functions
function [q, qd, qdd] = get_desired_traj(beta, t, duration)

    if nargin < 3
        duration = 1;
    end

    [B, dB, ddB] = Bezier_kernel_deg5(t/duration); %t/dur

    q = zeros(7,length(t));
    qd = zeros(7,length(t));
    qdd = zeros(7,length(t));
    
    for j = 1:6
        q = q + beta{j} .* B(:,j)';
        qd = qd + beta{j} .* dB(:,j)';
        qdd = qdd + beta{j} .* ddB(:,j)';
    end

    qd = qd / duration;
    qdd = qdd / duration / duration;
end
