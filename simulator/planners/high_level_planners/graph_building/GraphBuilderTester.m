
clear all;
close all;
clc;

%% Generate List of Configs a Known Distance Apart

num_points = 100;
base_angle = linspace(0,2*pi-0.0001,num_points);
for i = 1:num_points
    q(:,i) = [base_angle(i);-pi/2;0;0;0;0;0];
end

%% Check Distance

for i = 1:num_points
    for j = 1:num_points
        if i ~= j
            q_dist(i,j) = norm(angdiff(q(:,i)-q(:,j)));
        else
            q_dist(i,j) = inf;
        end
    end
end

num_valid_edges = sum(sum(q_dist <= 0.3))

%% Save as Space Separated .txt File

writematrix(q','SpacedConfigurations_v4.txt','Delimiter',' ')