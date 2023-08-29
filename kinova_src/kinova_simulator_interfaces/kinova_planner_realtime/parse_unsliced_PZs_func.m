function [unsliced_f_c_PZs,unsliced_n_c_PZs] = parse_unsliced_PZs_func()
%% read output from file

unsliced_f_c_int = readmatrix('unsliced_f_c_int.out', 'FileType', 'text');
unsliced_n_c_int = readmatrix('unsliced_n_c_int.out', 'FileType', 'text');

num_time_steps = 40;

%% parse output to zonotopes

ind = find(unsliced_f_c_int(:,1) == 123456);
assert(length(ind) == num_time_steps);
unsliced_f_c_PZs = cell(length(ind),1);
start_ind = 1;
for i = 1:length(ind)
    unsliced_f_c_PZs{i} = zonotope( unsliced_f_c_int(start_ind : ind(i)-1, :)' );
    start_ind = ind(i)+1;
end

ind = find(unsliced_n_c_int(:,1) == 123456);
assert(length(ind) == num_time_steps);
unsliced_n_c_PZs = cell(length(ind),1);
start_ind = 1;
for i = 1:length(ind)
    unsliced_n_c_PZs{i} = zonotope( unsliced_n_c_int(start_ind : ind(i)-1, :)' );
    start_ind = ind(i)+1;
end

%% plot zonotopes

% hold on; axis equal;
% 
% % plot x,y component of f
% for i = [1:10:num_time_steps, num_time_steps]
%     plot(unsliced_f_c_PZs{i}, [1,2]);
% end

end