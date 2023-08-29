%% Configuration Graph Planner
% Zachary Brei
% 03/19/2023

% Use a graph of configurations q to generate a list of waypoint
% configurations between a start and goal configuration that are collision
% free.

clear all;
close all;
clc;

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

%% Define Start and Goal and K_range

% k_range
k_range = pi/72;

% simple rotation
start = [0; -pi/2; 0; 0; 0; 0; 0];
goal = [pi/4; -pi/2; 0; 0; 0; 0; 0];

% random start and goal
% needs state limits
start = randn(7,1).*params.nominal.

%% Load Configuration Graph

% Note that these need to match and in future store the configuration and
% collision checking points in each node. 

config_struct = load('PlannerGraphResult2.mat');
q_valid_list = config_struct.q_valid_list;

graph_struct = load('QGraph700kPartial.mat');
Q_Graph = graph_struct.Q_Graph;

%% Reduce Graph to Largest Connected Component

[bin, binsize] = conncomp(Q_Graph,'Type','weak');

idx = (binsize(bin) == max(binsize));

Q_Graph_Reduced = subgraph(Q_Graph,idx);
q_valid_list_reduced = q_valid_list(:,idx);

%% Generate Path

% collision checking to remove nodes in collision

% find closest nodes to start and goal? assign as first and last waypoints?
% or just add start and goal as nodes, assuming they are collision free?
% but they wouldn't be connected.

% iterate through the whole graph and check for closest node? or check for
% node within a certain range?
start_dist = inf;
start_node = NaN;
goal_dist = inf;
goal_node = NaN;

for i = 1:length(q_valid_list_reduced)

    if norm(q_valid_list_reduced(:,i)-start) < start_dist
        start_dist = norm(q_valid_list_reduced(:,i)-start);
        start_node = i;
    end
    if norm(q_valid_list_reduced(:,i)-goal) < goal_dist
        goal_dist = norm(q_valid_list_reduced(:,i)-goal);
        goal_node = i;
    end

end

if start_dist > 7*k_range || goal_dist > 7*k_range
    fprintf('Start or Goal Node are Far Away!!!')
end
if goal_node == start_node
    fprintf('Start and Goal Nodes are the Same!!!')
end

% generate path
[Path, dist, waypoint_nodes] = shortestpath(Q_Graph_Reduced,start_node,goal_node);


% store waypoints 
% store them in the agent? or wherever the HLP is called so that they can
% just be pulled out. 

%% Simulate Robot