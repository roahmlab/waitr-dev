%% Kinova Hardware Trajectory Animation

clear all;
close all;
clc;

f1 = figure(1);
% file name to save the gif as
traj_save_file = 'SimulationGif_Trajectory';
% create video writer objects
traj_vid = VideoWriter(traj_save_file);
% setting the frame rate
framerate = 50;
traj_vid.FrameRate = framerate;
% setting quality
Quality = 100;
traj_vid.Quality = Quality;


%% Parameters

duration = 1.75; % seconds

% for plotting
brake_face_alpha = 0.2;
brake_face_color = 'r';
jitter_face_color = 'm';
fontsize = 20;
linewidth = 2;

%% Load Data

load('Simulation_v2.mat');
agent_urdf = 'Kinova_Grasp_w_Tray.urdf';

%% Extract Data

% for nominal values
time = A.time;
pos = A.state(A.joint_state_indices,:);
vel = A.state(A.joint_speed_indices,:);
accel = A.reference_acceleration;
% control_torque = rosbag.control_torque;

% for desired values
% pos_des = rosbag.debug_q_des;
% vel_des = rosbag.debug_qd_des;
% accel_des = rosbag.debug_qdd_des;

% for reach set values
% rs_duration = rosbag.debug_duration; % for determining if braking maneuver occured
% rs_pos = rosbag.traj_pos;
% rs_vel = rosbag.traj_vel;
% rs_accel = rosbag.traj_accel;
% rs_opt_k = rosbag.traj_k;

%% Load Robot Parameters

% match with Hardware experiment settings
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

%% Calculating Acceleration

% % actuator inertia: match with hardware experiment settings
% transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; 
% 
% joint_angles = pos';
% joint_angular_velocity = vel';
% input = control_torque';
% 
% qdd_post = zeros(7,length(time)); % accel'; % 
%     % calculating the acceleration in post to compare with what is stored
% parfor i = 1:length(time(1:end-1))
%     [M, C, g] = calculate_dynamics(joint_angles(:,i)', joint_angular_velocity(:,i)', params.true);
%     for j = 1:size(M,1)
%         M(j,j) = M(j,j) + transmision_inertia(j);
%     end
%     qdd_post(:,i) = M\(input(:,i+1)-C*joint_angular_velocity(:,i)-g);
% end



%% RNEA

% % calling rnea
% Tau = cell(1,length(pos));
% F = cell(1,length(pos));
% N = cell(1,length(pos));
% force = zeros(3,length(pos));
% moment = zeros(3,length(pos));
% parfor i = 1:length(pos)
% 
% %     % clear relevant variables
% %     clear tau f n
% 
%     % call rnea
%     [tau, f, n] = rnea(joint_angles(:,i)',joint_angular_velocity(:,i)',joint_angular_velocity(:,i)',d_postd_post(:,i)',true,params.true); % A.acceleration(:,i)'
% 
%     % store rnea results
%     Tau{i} = tau;
%     F{i} = f;
%     N{i} = n;
% 
%     % store the contact forces
%     force(:,i) = f(:,10);
%     % store the contact moments
%     moment(:,i) = n(:,10);
% 
% end

%% Plotting Braking Patches

% counter = 0;
% for i = 1:68 % length(rs_opt_k)
% 
%     subplot(3,1,1)
%     hold on
%     plot([duration*0.5*i duration*0.5*i], [-6 6],'--r')
%     subplot(3,1,2)
%     hold on
%     plot([duration*0.5*i duration*0.5*i], [-6 6],'--r')
%     subplot(3,1,3)
%     hold on
%     plot([duration*0.5*i duration*0.5*i], [-6 6],'--r')
%     
%     if sum(rs_opt_k(i,:)) == 0
%         counter = counter + 1;
% 
%         if i == 54
%             counter = counter - 1;
%         end
% 
%         time_window = (i+counter) * duration*0.5;
%         subplot(3,1,1)
%         hold on
%         patch([time_window time_window time_window+duration*0.5 time_window+duration*0.5],[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',brake_face_color,'EdgeAlpha',0)
%         subplot(3,1,2)
%         hold on
%         patch([time_window time_window time_window+duration*0.5 time_window+duration*0.5],[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',brake_face_color,'EdgeAlpha',0)
%         subplot(3,1,3)
%         hold on
%         patch([time_window time_window time_window+duration*0.5 time_window+duration*0.5],[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',brake_face_color,'EdgeAlpha',0)
% 
%     end
% 
% end

%% Plotting Jitter Patches

% % patch 1
% time_window = [19.25 19.25 30.625 30.625];
% subplot(3,1,1)
% hold on
% patch(time_window,[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',jitter_face_color,'EdgeAlpha',0)
% subplot(3,1,2)
% hold on
% patch(time_window,[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',jitter_face_color,'EdgeAlpha',0)
% subplot(3,1,3)
% hold on
% patch(time_window,[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',jitter_face_color,'EdgeAlpha',0)
% 
% % patch 2
% time_window = [39.375 39.375 53.375 53.375];
% subplot(3,1,1)
% hold on
% patch(time_window,[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',jitter_face_color,'EdgeAlpha',0)
% subplot(3,1,2)
% hold on
% patch(time_window,[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',jitter_face_color,'EdgeAlpha',0)
% subplot(3,1,3)
% hold on
% patch(time_window,[-6 6 6 -6],brake_face_color,'FaceAlpha',brake_face_alpha,'FaceColor',jitter_face_color,'EdgeAlpha',0)


%% Plotting Base Trajectories

% plot position trajectories
subplot(3,1,1)
hold on
plot(time,pos,'LineWidth',linewidth)

% plot velocity trajectories
subplot(3,1,2)
hold on
plot(time,vel,'LineWidth',linewidth)

% plot acceleration trajectories
subplot(3,1,3)
hold on
plot(time,accel,'LineWidth',linewidth)

%% Animate Plots

time_end = time(end); % seconds
framerate = 50; % fps

time_animation = 0:1/framerate:time_end;

for i = 1:length(time_animation)

    if i > 1
        delete(pos_line)
        delete(vel_line)
        delete(accel_line)
    end

    subplot(3,1,1)
    hold on
    pos_line = plot([time_animation(i) time_animation(i)],[-6 6],'-k','LineWidth',linewidth);
    subplot(3,1,2)
    hold on
    vel_line = plot([time_animation(i) time_animation(i)],[-0.15 0.15],'-k','LineWidth',linewidth);
    subplot(3,1,3)
    hold on
    accel_line = plot([time_animation(i) time_animation(i)],[-0.4 0.4],'-k','LineWidth',linewidth);

    % plot formatting
    subplot(3,1,1)
    title('Joint Position')
    ylabel('rad')
    xlim([0,time_end])
    ylim([-6 6])
    set(gca,'FontSize',fontsize)
    set(gcf,'color','w')
    
    subplot(3,1,2)
    title('Joint Velocity')
    ylabel('rad/s')
    xlim([0,time_end])
    ylim([-0.15 0.15])
    set(gca,'FontSize',fontsize)
    set(gcf,'color','w')
    
    subplot(3,1,3)
    title('Joint Acceleration')
    xlabel('Time (s)')
    ylabel('rad/s^2')
    xlim([0,time_end])
    ylim([-0.4 0.4])
    set(gca,'FontSize',fontsize)
    set(gcf,'color','w')

    if i == 1
        fprintf("Paused, re-adjust plot size.")
        pause()
        % open the files
        open(traj_vid)
    end

    make_video(f1,traj_vid)

end

close(traj_vid)

%% Helper Functions

function [M, C, g] = calculate_dynamics(q, qd, params)
            M = rnea_mass(q, params);
            C = rnea_coriolis(q, qd, params);
            g = rnea_gravity(q, params);
end

function make_video( h,video )
    frame = getframe(h);
    writeVideo(video,frame)
end