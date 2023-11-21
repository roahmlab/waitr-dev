% %% description
% This script gets the statics of the summaries generated from
% kinova_run_100_worlds
%
% Authors: Bohao Zhang (adapted from Shreyas Kousik code)
% Created 25 October 2022

clear; clc;

use_robust_input = true;

save_file_header = 'trial_' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/longer_duration_pi72_parallel_03022023' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/longer_duration_pi72_03022023' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/longer_duration_02252023' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/dur2s_largeStateBuffer_noObs_03052023' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/dur2s_largeStateBuffer_noObs_03052023_v2' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/dur2s_largeStateBuffer_noObs_03052023_v2_single' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/dur2s_largeStateBuffer_10Obs_03072023' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/dur2s_largeStateBuffer_10Obs_03082023'; % paper result
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/dur2s_largeStateBuffer_10Obs_03082023_graph' ;
% file_location = '/home/roahmlab/Documents/armour-dev/kinova_src/results/rtd-force/dur2s_largeStateBuffer_10Obs_03082023_graph_armour' ;
% file_location = 'D:\Grad School\Research\Roahm Lab\RTD Force\dur2s_largeStateBuffer_10Obs_03082023\dur2s_largeStateBuffer_10Obs_03082023';
% file_location = 'D:\Grad School\Research\Roahm Lab\RTD Force\dur2s_largeStateBuffer_10Obs_03082023_graph\dur2s_largeStateBuffer_10Obs_03082023_graph';
% file_location = 'D:\Grad School\Research\Roahm Lab\RTD Force\dur2s_largeStateBuffer_10Obs_03082023_graph_armour\dur2s_largeStateBuffer_10Obs_03082023_graph_armour';
% file_location = '/home/roahmlab/Documents/waitr-dev/kinova_src/results/rtd-force/k1234567_pi48_10Obs_11102023' ;
% file_location = '/home/roahmlab/Documents/waitr-dev/kinova_src/results/rtd-force/k1234567_pi32_10Obs_11102023' ;
% file_location = '/home/roahmlab/Documents/waitr-dev/kinova_src/results/rtd-force/k1234567_pi32_10Obs_11102023_graph' ;
% file_location = '/home/roahmlab/Documents/waitr-dev/kinova_src/results/rtd-force/k1234567_pi48_10Obs_11102023_graph' ;
% file_location = '/home/roahmlab/Documents/waitr-dev/kinova_src/results/rtd-force/k1234567_pi48_nt128_10Obs_11152023_fixed_graph' ;
% file_location = '/home/roahmlab/Documents/waitr-dev/kinova_src/results/rtd-force/k1234567_pi32_nt128_10Obs_11152023_fixed_graph' ;
% file_location = '/home/roahmlab/Documents/waitr-dev/kinova_src/results/rtd-force/k123_pi32_k4567_pi72_nt128_10Obs_11152023_fixed_graph';
file_location = '/home/roahmlab/Documents/waitr-dev/kinova_src/results/rtd-force/k1234567_pi72_10Obs_11102023_ARMOUR_SLP' ;

% file_location = '../results/hard' ;
addpath(file_location);

summary_files = dir([file_location, '/trial_*']);

collision_check = [];
input_check = [];
ultimate_bound_check = [];
joint_limit_check = [];
goal_check = [];
infeasible_check = [];
grasp_separation_check = [];
grasp_slipping_check = [];
grasp_tipping_check = [];
maxiter = [];
no_move = [];
mean_planning_time = [];
mean_vel = [];

num_total_iterations = [];

for i = 1:length(summary_files)
    data = load(summary_files(i).name);

    mean_planning_time = [mean_planning_time mean(cell2mat(data.P.info.planning_time))];
    num_total_iterations = [num_total_iterations data.summary.total_iterations];
    mean_vel(:,i) = mean(abs(data.summary.trajectory(data.A.joint_speed_indices,:)),2);

    if data.summary.total_iterations > data.S.max_sim_iterations
        maxiter = [maxiter i];
    end

    if data.summary.total_iterations - 1 == data.S.stop_threshold
        no_move = [no_move i];
    end

    if data.summary.collision_check
        collision_check = [collision_check, i];
%         continue;
    end
    if data.summary.input_check
        input_check = [input_check, i];
%         continue;
    end
    if data.summary.ultimate_bound_check
        ultimate_bound_check = [ultimate_bound_check, i];
%         continue;
    end
    if data.summary.joint_limit_check
        joint_limit_check = [joint_limit_check, i];
%         continue;
    end
    if data.summary.grasp_separation_check
        grasp_separation_check = [grasp_separation_check i];
    end
    if data.summary.grasp_slipping_check
        grasp_slipping_check = [grasp_slipping_check i];
    end
    if data.summary.grasp_tipping_check
        grasp_tipping_check = [grasp_tipping_check i];
    end
    if data.summary.goal_check
        goal_check = [goal_check, i];
        continue;
    end

    infeasible_check = [infeasible_check, i];

    if ismember(i,infeasible_check)
        fprintf('%d %d %d\n\n', i, data.summary.goal_check, data.summary.total_iterations);

%         data.summary.total_simulated_time
%         data.summary.total_real_time
%         sum(data.summary.stop_check)
    end

end

fprintf("Test Summary\n");
fprintf("Total Number of Test Trials: %d\n", length(summary_files));
fprintf("Number of Test Trials that collision occurs: %d\n", length(collision_check));
fprintf("Number of Test Trials that exceed torque limits: %d\n", length(input_check));
fprintf("Number of Test Trials that tracking error exceeds ultimate bound: %d\n", length(ultimate_bound_check));
fprintf("Number of Test Trials that exceed joint (position/velocity) limits: %d\n", length(joint_limit_check));
fprintf("Number of Test Trials that violated separation constraint: %d\n",length(grasp_separation_check));
fprintf("Number of Test Trials that violated slipping constraint: %d\n",length(grasp_slipping_check));
fprintf("Number of Test Trials that violated tipping constraint: %d\n",length(grasp_tipping_check));
fprintf("Number of Test Trials that reach the goals: %d\n", length(goal_check));
fprintf("Number of Test Trials that do not reach the goals but stop safely: %d\n", length(infeasible_check));
fprintf("Average number of iterations: %d\n",mean(num_total_iterations))

%% Velocity Processing
mean_vel;

% find mean velocity of trials that move
idx = 1:length(summary_files);
idx_move = [];
for i=1:length(idx)
    if ~ismember(i,no_move)
        idx_move = [idx_move i];
    end
end

mean_vel_processed = mean_vel(:,idx_move);
overall_mean = mean(mean_vel_processed,2)