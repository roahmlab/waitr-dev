%% Velocity vs K-Range Plotting

clear all;
close all;
clc;

%% Plotting Settings

markersize = 8;

fig_num = 0;

%% Load in the Data

load('Velocity_Krange_Test.mat');

%% Get the Field Names

field_names = fieldnames(velocity_test_w_krange);

%% Iterate through to pull out the data

for i = 1:length(field_names)

    mean_velocity(:,i) = velocity_test_w_krange.(field_names{i}).vel_mean_max(:,1);
    max_velocity(:,i) = velocity_test_w_krange.(field_names{i}).vel_mean_max(:,2);
    krange(:,i) = velocity_test_w_krange.(field_names{i}).krange;
    goal_check(i) = velocity_test_w_krange.(field_names{i}).goal_check;
end

%% Plotting

% test indices
start_1 = 1;
end_1 = 12;
start_2 = end_1+1;
end_2 = 20;
start_3 = end_2+1;

fig_num = fig_num + 1;
figure(fig_num)
num_plots = size(krange,1);
% plotting all joint velocities vs krange
for i = 1:num_plots

    % plot the max velocity
    subplot(2,num_plots,i)
    hold on
    plot(krange(i,start_1:end_1),max_velocity(i,start_1:end_1),'ok','MarkerSize',markersize)
    plot(krange(i,start_2:end_2),max_velocity(i,start_2:end_2),'*g','MarkerSize',markersize)
    plot(krange(i,start_3:end),max_velocity(i,start_3:end),'diamondc','MarkerSize',markersize)
    title('Max Velocity vs K-Range for qd1')
    legend('u_s=0.6','u_s=0.75','u_s=0.85')
    for j = 1:size(krange,2)
        if ~goal_check(j)
            plot(krange(i,j),max_velocity(i,j),'xr')
        end
    end

    % plot the mean velocity
    subplot(2,num_plots,num_plots+i)
    hold on
    plot(krange(i,start_1:end_1),mean_velocity(i,start_1:end_1),'ok','MarkerSize',markersize)
    plot(krange(i,start_2:end_2),mean_velocity(i,start_2:end_2),'*g','MarkerSize',markersize)
    plot(krange(i,start_3:end),mean_velocity(i,start_3:end),'diamondc','MarkerSize',markersize)
    title('Max Velocity vs K-Range for qd1')
    legend('u_s=0.6','u_s=0.75','u_s=0.85')
    for j = 1:size(krange,2)
        if ~goal_check(j)
            plot(krange(i,j),mean_velocity(i,j),'xr')
        end
    end

end


plot_ticks = fliplr({pi/10,pi/15,pi/20,pi/25,pi/30,pi/35,pi/40,pi/45,pi/50,pi/55,pi/60,pi/65,pi/70,pi/75});
plot_tick_labels = fliplr({'\pi/10','\pi/15','\pi/20','\pi/25','\pi/30','\pi/35','\pi/40','\pi/45','\pi/50','\pi/55','\pi/60','\pi/65','\pi/70','\pi/75'});

% plot the velocity vs k_range

fig_num = fig_num + 1;
figure(fig_num)

% subplot(2,2,1)
% hold on
% plot(krange(1,start_1:end_1),max_velocity(1,start_1:end_1),'ok','MarkerSize',markersize)
% plot(krange(1,start_2:end_2),max_velocity(1,start_2:end_2),'*g','MarkerSize',markersize)
% plot(krange(1,start_3:end),max_velocity(1,start_3:end),'diamondc','MarkerSize',markersize)
% title('Max Velocity vs K-Range for qd1')
% % xticks(plot_ticks)
% % xticklabels(plot_tick_labels)
% legend('u_s=0.6','u_s=0.75','u_s=0.85')
% for j = 1:size(krange,2)
%     if ~goal_check(j)
%         plot(krange(1,j),max_velocity(1,j),'xr')
%     end
% end
% 
% subplot(2,2,2)
% hold on
% plot(krange(2,start_1:end_1),max_velocity(2,start_1:end_1),'ok','MarkerSize',markersize)
% plot(krange(2,start_2:end_2),max_velocity(2,start_2:end_2),'*g','MarkerSize',markersize)
% plot(krange(2,start_3:end),max_velocity(2,start_3:end),'diamondc','MarkerSize',markersize)
% title('Max Velocity vs K-Range for qd2')
% legend('u_s=0.6','u_s=0.75','u_s=0.85')
% for j = 1:size(krange,2)
%     if ~goal_check(j)
%         plot(krange(2,j),max_velocity(2,j),'xr')
%     end
% end

subplot(1,3,1)
hold on
plot(krange(1,start_1:end_1),mean_velocity(1,start_1:end_1),'ok','MarkerSize',markersize)
plot(krange(1,start_2:end_2),mean_velocity(1,start_2:end_2),'*g','MarkerSize',markersize)
plot(krange(1,start_3:end),mean_velocity(1,start_3:end),'diamondb','MarkerSize',markersize)
title('Mean Velocity vs K-Range for qd1')
xlabel('k-range (rad)')
ylabel('mean velocity (rad/s)')
ylim([0 0.15])
legend('u_s=0.6','u_s=0.75','u_s=0.85')
for j = 1:size(krange,2)
    if ~goal_check(j)
        plot(krange(1,j),mean_velocity(1,j),'xr')
    end
end

subplot(1,3,2)
hold on
plot(krange(2,start_1:end_1),mean_velocity(2,start_1:end_1),'ok','MarkerSize',markersize)
plot(krange(2,start_2:end_2),mean_velocity(2,start_2:end_2),'*g','MarkerSize',markersize)
plot(krange(2,start_3:end),mean_velocity(2,start_3:end),'diamondb','MarkerSize',markersize)
title('Mean Velocity vs K-Range for qd2')
xlabel('k-range (rad)')
ylabel('mean velocity (rad/s)')
legend('u_s=0.6','u_s=0.75','u_s=0.85')
ylim([0 0.15])
for j = 1:size(krange,2)
    if ~goal_check(j)
        plot(krange(2,j),mean_velocity(2,j),'xr')
    end
end


subplot(1,3,3)
hold on
plot(krange(6,start_1:end_1),mean_velocity(6,start_1:end_1),'ok','MarkerSize',markersize)
plot(krange(6,start_2:end_2),mean_velocity(6,start_2:end_2),'*g','MarkerSize',markersize)
plot(krange(6,start_3:end),mean_velocity(6,start_3:end),'diamondb','MarkerSize',markersize)
title('Mean Velocity vs K-Range for qd6')
xlabel('k-range (rad)')
ylabel('mean velocity (rad/s)')
legend('u_s=0.6','u_s=0.75','u_s=0.85')
ylim([0 0.15])
for j = 1:size(krange,2)
    if ~goal_check(j)
        plot(krange(6,j),mean_velocity(6,j),'xr')
    end
end
