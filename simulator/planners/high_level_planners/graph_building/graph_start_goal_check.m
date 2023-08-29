%% Graph Checker for Start/Goal Pair

clear all;
close all;
clc;

%% Define Start and Goal to Check

% define base joint offset rotation
offset = 0;

% define start and goal configurations

% Ford Hardware Demo Configurations
start = [-pi/6+offset;-pi/2;-pi/2;pi/2;0;pi/2;pi/2];
goal = [pi/6+offset;-pi/2;pi/2;pi/2;pi;-pi/2;pi/2];

% define threshold for how close the closest node should be to the start
% and goal
start_goal_distance_threshold = 0.5;

%% Load Robot

agent_urdf = 'Kinova_Grasp_URDF.urdf';
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];

%% Load Joint Positions

% path from /scripts to /Graphs
sample_nodes = load('../../Graphs/uniformNodes_hardware.csv');
% sample_nodes = load('../../Graphs/QConfig_uniform_tiltrand_v2.txt');

% wrapping to [-pi pi]
sample_nodes(:,1) = wrapToPi(sample_nodes(:,1));
% check if proper structure HLP.sample_nodes.q_valid_list
if class(sample_nodes) == 'double'
    tmp_sample_nodes = sample_nodes;
    clear sample_nodes
    sample_nodes.q_valid_list = tmp_sample_nodes;
    clear tmp_sample_nodes
end
% code expects sample nodes to be 7 x n array (where n>7)
if size(sample_nodes.q_valid_list,2) < size(sample_nodes.q_valid_list,1)
    sample_nodes.q_valid_list = sample_nodes.q_valid_list';
end

%% Load In Adjacency Matrix

% read adjacency matrix file
adj_matrix_sparse_data = readmatrix('../../Graphs/adj_matrix_uniform_mult5.csv');
% adj_matrix_sparse_data = readmatrix('../../Graphs/adj_matrix_uniform_tiltrand_v2_range0p6.txt');

% form sparse matrix from adjacency matrix
adj_matrix_sparse = sparse(adj_matrix_sparse_data(1:end,1)+1, ...
                           adj_matrix_sparse_data(:,2)+1, ...
                           adj_matrix_sparse_data(:,3), ...
                           size(sample_nodes.q_valid_list,2), size(sample_nodes.q_valid_list,2));

%% Build Graph

G = graph(adj_matrix_sparse, 'lower');

%% Process Graph

% get largest connected subgraph
[bins, binsize] = conncomp(G);
[~, max_id] = max(binsize);
G_maxconn = subgraph(G, bins == max_id);

%% Find Closest Nodes

% get joint configurations of the connected subgraph
q_subgraph = sample_nodes.q_valid_list(:, bins == max_id);
% calculate distances between nodes and start/goal
difference_to_goal = vecnorm(wrapToPi(goal - q_subgraph));
difference_to_start = vecnorm(wrapToPi(start - q_subgraph));
% find the closest
[end_diff, end_idx] = min(difference_to_goal)
[start_diff, start_idx] = min(difference_to_start)

% check if farther than threshold
if end_diff > start_goal_distance_threshold
    fprintf('    HLP: Goal node is far away!!! Distance: %f\n', end_diff)
end
if start_diff > start_goal_distance_threshold
    fprintf('    HLP: Start node is far away!!! Distance: %f\n', start_diff)
end

%% Find the Shortest Path Between Start and Goal

[path, len] = shortestpath(G_maxconn, start_idx, end_idx);

if isempty(path)
    error('can not find any path!');
end

graph_waypoints = q_subgraph(:,path);

%% Plot Waypoints

for i = 1:size(graph_waypoints,2)

    show(robot,graph_waypoints(:,i))
    pause(0.5)
    if i == 1
        pause()
    end
end