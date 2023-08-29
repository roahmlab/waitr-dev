
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

%% Generate Random Configurations

num_tries = 1000000;
q_list = nan(7,num_tries);

% counter = 0;
% while counter < 10000
parfor i = 1:num_tries

    % create random q
    q_rand = randomConfiguration(robot);

    q_aug = [q_rand; 0; 0; 0];

    % generate forward kinematics of random configuration
    q_fk_res = forward_kinematics(q_aug, params.nominal.T0, params.nominal.joint_axes);

    % get the euler angles of the random configuration
%     q_ee_euler_angles = tform2eul(q_fk_res);
    q_ee_euler_angles = rotm2eul(q_fk_res(1:3,1:3), 'XYZ');

%     % if valid, add to list of q's
    if abs(q_ee_euler_angles(1))<0.05 & abs(q_ee_euler_angles(2))<0.05 % & abs(q_ee_euler_angles(3))<0.03
        q_list(:,i) = q_rand;
    end

    % increment counter
%     counter = counter + 1;

end

%% Post Process

num_valid_q = 0;
for i = 1:num_tries
    check = ~isnan(q_list(1,i));
    if check
        num_valid_q = num_valid_q + 1;
    end
end
num_valid_q

idx_valid = find(~isnan(q_list(1,:)));

q_valid_list = q_list(:,idx_valid);

% 
% fail = false;
% for i = 1:length(idx_valid)
%     [u,f,n] = rnea(q_list(:,idx_valid(i)),zeros(7,1),zeros(7,1),zeros(7,1),true,params.nominal)
%     
%     fx = f(1,10);
%     fy = f(2,10);
%     fz = f(3,10);
%     
%     % ZMP_Moment = n(:,10) + cross([0;0;cup_height],f(:,10));
%     
%     sep = -1*fz; %fz; %
%     slip = sqrt(fx^2+fy^2) - u_s*abs(fz) + 0.65; % offset to make initial/goal closer to horizontal
%     ZMP = cross([0;0;1],n(:,10))./dot([0;0;1],f(:,10));
%     %             ZMP = cross(ZMP_Moment,[0;0;1])./dot([0;0;1],f(:,10)); % RNEA
%     %             passes out the force and moment at the joint so original ZMP
%     %             was correct
%     tip = sqrt(ZMP(1)^2 + ZMP(2)^2) - surf_rad; % + tip_threshold;
%     
%     if (sep > 0) || (slip > 0) || (tip > 0) % greater than zero is violation
%         fail = true
%         break
%     end
% end
