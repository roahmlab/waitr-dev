
clear all; 
close all;
clc;

% conncomp

%% Load Data

qs = load('PlannerGraphResult1.mat')
q_valid_list = qs.q_valid_list;

%% Graph Parameters

k_range = pi/72;
edge_threshold = 7*k_range;

%% Create Graph

Q_Graph = graph();
e = Q_Graph.Edges;
Q_Graph = addnode(Q_Graph,length(q_valid_list));
% plot(Q_Graph)

%% Iterate Through q List and Add Edges Where Appropriate
for i = 1:1:round(length(q_valid_list)/1)
    for j = i:1:round(length(q_valid_list)/1)

        if i ~= j

            % Check distance from current i to current j
            q_dist = norm(q_valid_list(:,i)-q_valid_list(:,j));

            % if distance is below threshold, add edge
            if q_dist < edge_threshold
                Q_Graph = addedge(Q_Graph,i,j);
            end

        end

    end
end

plot(Q_Graph)

%% Saving Graph

save('Q_Graph_70kNodes_7krange.mat','Q_Graph')