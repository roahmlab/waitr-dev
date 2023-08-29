clear; clc;

%% read output from file
unsliced_f_c_int = readmatrix('unsliced_f_c_int.out', 'FileType', 'text');
unsliced_n_c_int = readmatrix('unsliced_n_c_int.out', 'FileType', 'text');
force_reachset_values = readmatrix('buffer/armour_wrench_values.out', 'FileType', 'text');
force_constraint_values = readmatrix('buffer/armour_force_constraint_radius.out', 'FileType', 'text');

num_time_steps = 100;

%% parse output to zonotopes
f_rs_c = force_reachset_values(:,1:3);
n_rs_c = force_reachset_values(:,4:6);
f_rs_r = force_reachset_values(:,7:9);
n_rs_r = force_reachset_values(:,10:12);
% sep_ub_cuda = force_constraint_values(1:100,1);
% slip_ub_cuda = force_constraint_values(101:200,1);
% tip_ub_cuda = force_constraint_values(201:300,1);
% sep_lb_cuda = force_constraint_values(1:100,2);
% slip_lb_cuda = force_constraint_values(101:200,2);
% tip_lb_cuda = force_constraint_values(201:300,2);

sliced_f = cell(num_time_steps,1);
sliced_n = cell(num_time_steps,1);
for i = 1:num_time_steps
    sliced_f{i} = zonotope(f_rs_c(i,:)', diag(f_rs_r(i,:)));
    sliced_n{i} = zonotope(n_rs_c(i,:)', diag(n_rs_r(i,:)));
end

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
figure; hold on; axis equal;
% plot x,y component of f
for i = [1:10:num_time_steps, num_time_steps]
    plot(unsliced_f_c_PZs{i}, [1,2], 'b');
    plot(sliced_f{i}, [1,2], 'r');
end
title('f_x and f_y');

figure; hold on; axis equal;
% plot x,y component of n
for i = [1:10:num_time_steps, num_time_steps]
    plot(unsliced_n_c_PZs{i}, [1,2], 'b');
    plot(sliced_n{i}, [1,2], 'r');
end
title('n_x and n_y');