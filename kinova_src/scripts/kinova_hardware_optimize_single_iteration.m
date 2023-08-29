
clear all;
close all;
clc;

%% Load Data

load('HardwareVideo_MultipleTrials_05_17_2023_ROSData.mat')

%% Get Initial Conditions

% iterate through the traj info to find k values of zero, then take the
% corresponding initial conditions and desired position

%% Helper Functions

% Optimize Function: Calls CUDA code to find k_value given initial
% conditions.
function [plot_info_struct] = optimize(q_0, q_dot_0, q_ddot_0, q_des)
    
    %% Calculate Unsliced Reachable Sets
    
    % organize input to cuda program
    kinova_test_folder_path = '/home/baiyuew/ROAHM/armour-dev/kinova_src';
    cuda_input_file = fopen([kinova_test_folder_path, '/kinova_simulator_interfaces/kinova_planner_realtime/buffer/armour.in'], 'w');  
    
    for ind = 1:length(q_0)
        fprintf(cuda_input_file, '%.10f ', q_0(ind));
    end
    fprintf(cuda_input_file, '\n');
    for ind = 1:length(q_dot_0)
        fprintf(cuda_input_file, '%.10f ', q_dot_0(ind));
    end
    fprintf(cuda_input_file, '\n');
    for ind = 1:length(q_ddot_0)
        fprintf(cuda_input_file, '%.10f ', q_ddot_0(ind));
    end
    fprintf(cuda_input_file, '\n');
    for ind = 1:length(q_des)
        fprintf(cuda_input_file, '%.10f ', q_des(ind));
    end
    
    fclose(cuda_input_file);
    
    % call cuda program in terminal
    % you have to be in the proper path!
    terminal_output = system('env -i bash -i -c "../kinova_simulator_interfaces/kinova_planner_realtime/rtd_force_main_v2" '); % rtd-force path

    if terminal_output == 0
        % read FRS information if needed
        link_frs_center = readmatrix('armour_joint_position_center.out', 'FileType', 'text');
        link_frs_generators = readmatrix('armour_joint_position_radius.out', 'FileType', 'text');
        control_input_radius = readmatrix('armour_control_input_radius.out', 'FileType', 'text');
        constraints_value = readmatrix('armour_constraints.out', 'FileType', 'text');
        contact_constraint_radii = readmatrix('armour_force_constraint_radius.out', 'FileType', 'text');
        wrench_radii = readmatrix('armour_wrench_values.out', 'FileType', 'text');
    else
        k_opt = nan;
    end
    
    %% Parse the Unsliced Output
    
    % parse the unsliced PZ output
    [force_PZ_unsliced,moment_PZ_unsliced] = parse_unsliced_PZs_func();

    %% Process Sliced Wrench PZ

    % get center and radii values of sliced PZs
    force_center = wrench_radii(:,1:3);
    moment_center = wrench_radii(:,4:6);
    force_radii = wrench_radii(:,7:9);
    moment_radii = wrench_radii(:,10:12);
    
    % getting upper and lower bounds for patch plotting
    force_lower = force_center - force_radii;
    force_upper = force_center + force_radii;
    
    %% Calculate ZMP Overapproximation 
    
    % unsliced
    for i = 1:size(wrench_radii,1)
        if size(moment_PZ_unsliced{i}.Z,2) == 2
            % add two columns of zeros
            moment_zono = polyZonotope_ROAHM(moment_PZ_unsliced{i}.Z(:,1),[moment_PZ_unsliced{i}.Z(:,2), zeros(3,2)],[]);
        elseif size(moment_PZ_unsliced{i}.Z,2) == 3
            % add one column of zeros
            moment_zono = polyZonotope_ROAHM(moment_PZ_unsliced{i}.Z(:,1),[moment_PZ_unsliced{i}.Z(:,2), moment_PZ_unsliced{i}.Z(:,3), zeros(3,1)],[]);
        else
            moment_zono = polyZonotope_ROAHM(moment_PZ_unsliced{i}.Z(:,1),moment_PZ_unsliced{i}.Z(:,2:end),[]);
        end
        % calculating the PZ form of the constraints
        ZMP_PZ_top = cross([0;0;1],moment_zono);
        ZMP_PZ_bottom = interval(force_PZ_unsliced{i}); % *[0,0,1]; % modified
    %     ZMP_PZ_bottom = interval(ZMP_PZ_bottom);
        ZMP_PZ_bottom_inf = ZMP_PZ_bottom.inf(3);
        ZMP_PZ_bottom_sup = ZMP_PZ_bottom.sup(3);
        % Note that this is not the entire overapproximated PZ because the
        % bottom has the .sup as well
        ZMP_PZ_inf = interval(ZMP_PZ_top) / ZMP_PZ_bottom_inf;
        ZMP_PZ_sup = interval(ZMP_PZ_top) / ZMP_PZ_bottom_sup;
        ZMP_unsliced_PZ{i} = convHull(ZMP_PZ_inf,ZMP_PZ_sup);
    end
    
    % sliced
    for i = 1:size(wrench_radii,1)
        force_int{i} = interval(force_center(i,3)-force_radii(i,3),force_center(i,3)+force_radii(i,3));
        moment_zono = polyZonotope_ROAHM(moment_center(i,:)',eye(3).*moment_radii(i,:)');
        % calculating the PZ form of the constraints
        ZMP_PZ_top = cross([0;0;1],moment_zono);
        ZMP_PZ_bottom = force_int{i}; % *[0,0,1]; % modified
        ZMP_PZ_bottom_inf = ZMP_PZ_bottom.inf;
        ZMP_PZ_bottom_sup = ZMP_PZ_bottom.sup;
        % Note that this is not the entire overapproximated PZ because the
        % bottom has the .sup as well
        ZMP_PZ_inf = interval(ZMP_PZ_top) / ZMP_PZ_bottom_inf;
        ZMP_PZ_sup = interval(ZMP_PZ_top) / ZMP_PZ_bottom_sup;
        ZMP_PZ{i} = convHull(ZMP_PZ_inf,ZMP_PZ_sup);
    end

    %% Storing Outputs
    plot_info_struct.force_PZ_unsliced = force_PZ_unsliced;
    plot_info_struct.moment_PZ_unsliced = moment_PZ_unsliced;
    plot_info_struct.force_lower = force_lower;
    plot_info_struct.force_upper = force_upper;
    plot_info_struct.ZMP_unsliced_PZ = ZMP_unsliced_PZ;
    plot_info_struct.ZMP_PZ = ZMP_PZ;

end