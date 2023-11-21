%% Minimum Mass Matrix Eigenvalue Sampling Script
% Zachary Brei
% 11/17/2023

clear all;
close all;
clc;

%% Load Robot

agent_urdf = 'Kinova_Grasp_Champagne_Edge.urdf';

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


%% Sample Random Configurations

% number of configurations to sample
num_configs = 1e7;
true_minimum_eig_value = inf;
inf_minimum_eig_value = inf;

for i = 1:num_configs

    % create random configuration to sample
    q = randomConfiguration(robot);

    % calculate the true and interval forms of the mass matrix at the
    % random configuration
    M_int = rnea_mass(q,params.interval);
    M_true = rnea_mass(q,params.true);

    % might want to calculate minimum mass matrix separately

    % adding transmission inertia to mass matrix
    % values are obtained through sysid of robot
    % eig_sup_w_inertia = eig(M_int.sup)
    transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151];

    for j = 1:size(M_true)
    
        M_int(j,j) = M_int(j,j) + transmision_inertia(j);
        M_true(j,j) = M_true(j,j) + transmision_inertia(j);
    
    end
    
    eig_inf_w_inertia = eig(M_int.inf);
    % eig_sup_w_inertia = eig(M_int.sup)
    eig_true_w_inertia = eig(M_true);

    true_minimum_eig_value = min(true_minimum_eig_value,min(eig_true_w_inertia));
    inf_minimum_eig_value = min(inf_minimum_eig_value,min(eig_inf_w_inertia));

end

final_min_eigs_true_inf = [true_minimum_eig_value, inf_minimum_eig_value]