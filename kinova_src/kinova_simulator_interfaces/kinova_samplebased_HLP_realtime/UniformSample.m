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

    q_sample = samples(1:4,i);
    tray_yaw_des = samples(5,i);

    q_ik = inverse_kinematics_flat_end_effector(A,zeros(3,1),[q_sample;0;0;0],tray_yaw_des);

    q_list(:,i) = [q_sample(1:4); q_ik];
end

%% Post Process

sum(~any(isnan(q_list)))

q_valid_list = q_list(:,~any(isnan(q_list)));

%% Add world starts and goals
world_file_header = 'scene';
world_file_folder = '../../saved_worlds/rtd-force/dur2s_largeStateBuffer_10Obs_03082023/';
world_file_location = sprintf('%s*%s*', world_file_folder, world_file_header);
world_file_list = dir(world_file_location);

for i = 1:length(world_file_list)
    world_filename = world_file_list(i).name;
    [start, goal, obstacles] = load_saved_world([world_file_folder world_filename]);

    q_valid_list = [q_valid_list, start, goal];
end

%% Add some random configurations so that 20000 divides total number of nodes
num_still_need = 20000 - mod(size(q_valid_list, 2), 20000);

num_valid = 0;
q_list = [];

while num_valid < num_still_need
    warning('off', 'MATLAB:nearlySingularMatrix');

    q_sample = (A.joint_state_limits(2,1:4) - A.joint_state_limits(1,1:4))' .* rand(4,1) + A.joint_state_limits(1,1:4)';
    tray_yaw_des = rand * 2 * pi - pi;

    q_ik = inverse_kinematics_flat_end_effector(A,zeros(3,1),[q_sample;0;0;0],tray_yaw_des);

    q = [q_sample(1:4); q_ik];

    if ~any(isnan(q))  
        num_valid = num_valid + 1;
        q_list = [q_list, q];
    end
end

sum(~any(isnan(q_list)))

q_valid_list = [q_valid_list, q_list];

%% Save Results
save('uniformNodes.mat', 'q_valid_list');

% write the transpose!
writematrix(q_valid_list', 'uniformNodes.csv', 'Delimiter', ' ');