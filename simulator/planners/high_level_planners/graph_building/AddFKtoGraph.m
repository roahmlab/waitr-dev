%% Graph Forward Kinematics For Nodes
clear all;
close all;
clc;

%% Load Graph

graph_struct = load('QGraph1mil_Config.mat');
Q_Graph = graph_struct.Q_Graph;

% Reduce Graph to Largest Connected Component

[bin, binsize] = conncomp(Q_Graph,'Type','weak');

idx = (binsize(bin) == max(binsize));

Q_Graph_Reduced = subgraph(Q_Graph,idx);

Q_Nodes = table2array(Q_Graph_Reduced.Nodes)';

%% Load Robot

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

% test = nan(2,length(Q_Nodes));
% 
% test_cells = num2cell(test); % ,length(Q_Nodes));
% 
% Q_Graph_Reduced.Nodes.Test = test_cells';

%% Loop Through Joints and Calculate, Then Store Forward Kinematics

for i = 1:length(Q_Nodes)

    % calculate forward kinematics
    q_cur_fk = forward_kinematics_modified([Q_Nodes(:,i);0;0;0],params.true.T0,params.true.joint_axes);

    % generate points between the joints to check for collition
    alpha = linspace(0,1,5); % this is the number of points including the two joints

    counter = 1;
    for j = 1:length(q_cur_fk)-1
        
        % calculate points along the link
        points = alpha.*q_cur_fk{j}(1:3,4) + (1-alpha).*q_cur_fk{j+1}(1:3,4);

        
        for k = 1:length(points)
            for l = 1:3
                points_storage(i,counter) = points(l,k);
                counter = counter + 1;
            end
        end

%         figure(101)
%         hold on
%         plot3(points(1,:),points(2,:),points(3,:),'ok')

    end

end

%% Add Collision Points to Graph

Q_Graph_Reduced.Nodes.CollisionPoints = points_storage;

%% Save Graph

save('QGraph1mil_AllInfo.mat','Q_Graph_Reduced')