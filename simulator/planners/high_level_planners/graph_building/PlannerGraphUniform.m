%% robot params:

u_s = 0.609382421; 
surf_rad =  0.058 / 2;

agent_urdf = 'Kinova_Grasp_URDF.urdf';

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
joint_speed_limits = [-1.3963, -1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218;
                       1.3963,  1.3963,  1.3963,  1.3963,  1.2218,  1.2218,  1.2218]; % matlab doesn't import these from urdf so hard code into class
joint_input_limits = [-56.7, -56.7, -56.7, -56.7, -29.4, -29.4, -29.4;
                       56.7,  56.7,  56.7,  56.7,  29.4,  29.4,  29.4]; % matlab doesn't import these from urdf so hard code into class
transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class
M_min_eigenvalue = 8.29938; % matlab doesn't import these from urdf so hard code into class

%%  

A = uarmtd_agent(robot, params);
A.joint_state_limits(1,[1,3,5,7]) = -pi;
A.joint_state_limits(2,[1,3,5,7]) = pi;

%% Generate Uniform Configurations

num_nodes_for_each_dim = 20;

ranges = cell(5,1);
for i = [1,3]
    ranges{i} = linspace(A.joint_state_limits(1,i), A.joint_state_limits(2,i), num_nodes_for_each_dim + 1);
    ranges{i} = ranges{i}(1:end-1);
end
for i = [2,4]
    ranges{i} = linspace(A.joint_state_limits(1,i), A.joint_state_limits(2,i), num_nodes_for_each_dim);
end
ranges{5} = linspace(-pi, pi, num_nodes_for_each_dim + 1);
ranges{5} = ranges{5}(1:end-1);

[joint1, joint2, joint3, joint4, tray_yaw] = ndgrid(ranges{1}, ranges{2}, ranges{3}, ranges{4}, ranges{5});

samples = [reshape(joint1, [1, num_nodes_for_each_dim^5]);
           reshape(joint2, [1, num_nodes_for_each_dim^5]);
           reshape(joint3, [1, num_nodes_for_each_dim^5]);
           reshape(joint4, [1, num_nodes_for_each_dim^5]);
           reshape(tray_yaw, [1, num_nodes_for_each_dim^5])];

q_list = nan(7,num_nodes_for_each_dim^5);

parfor i = 1:(num_nodes_for_each_dim^5)
    warning('off', 'MATLAB:nearlySingularMatrix');

    % create random q
    q_sample = samples(1:4,i);
    tray_yaw_des = samples(5,i);

    q_ik = inverse_kinematics_flat_end_effector(A,zeros(3,1),[q_sample;0;0;0],tray_yaw_des);

%     q_aug = [q_sample(1:4); q_ik; 0; 0; 0];

    % generate forward kinematics of random configuration
%     q_fk_res = forward_kinematics(q_aug, params.nominal.T0, params.nominal.joint_axes);

    % get the euler angles of the random configuration
%     q_ee_euler_angles = tform2eul(q_fk_res);
%     q_ee_euler_angles = rotm2eul(q_fk_res(1:3,1:3), 'XYZ');

%     % if valid, add to list of q's
%     if abs(q_ee_euler_angles(1))<0.05 & abs(q_ee_euler_angles(2))<0.05 % & abs(q_ee_euler_angles(3))<0.03
%         q_list(:,i) = [q_sample(1:4);q_ik];
%     end

    q_list(:,i) = [q_sample(1:4); q_ik];

    % increment counter
%     counter = counter + 1;

end

%% Post Process

sum(~any(isnan(q_list)))

q_valid_list = q_list(:,~any(isnan(q_list)));

save('uniformNodes.mat', 'q_valid_list');