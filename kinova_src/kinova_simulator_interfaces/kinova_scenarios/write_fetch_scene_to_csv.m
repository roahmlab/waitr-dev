function [ ] = write_fetch_scene_to_csv(W, filename)
% takes in an example world
% writes .csv to be used in MoveIt

% create a matrix M
% first row: agent initial state

n = length(W.start);

M(1, :) = W.start';
M(2, :) = W.goal';
M(3, :) = nan*ones(1, n);

for i = 1:W.N_obstacles
    if W.obstacles{i}.is_base_obstacle
        continue;
    else
        M(end+1, :) =  [W.obstacles{i}.center', W.obstacles{i}.side_lengths, nan*ones(1, n-6)];
    end
end

if ~exist('filename', 'var')
    filename = ('scene_1.csv');
end
csvwrite(filename, M);
 
end