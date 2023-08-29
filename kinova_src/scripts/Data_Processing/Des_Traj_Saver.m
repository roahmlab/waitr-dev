%% Saving Evenly Sampled Desired Trajectory for Python Rendering

clear all;
close all;
clc;

%% For Debugging

robot = importrobot('Kinova_Grasp_w_Tray.urdf');
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];

%% Load Data

load('HardwareVideo_MultipleRuns_06_15_2023_ROSBAG_Data.mat')

%% Extract the Data

time = rosbag.raw_time - rosbag.raw_time(1);
q_des = (rosbag.debug_q_des);

%% Interpolate the Data

framerate = 50;
test = linspace(0,1,framerate+1);
spacing = diff(test);
% time(end) = 
interp_time = 0:spacing(1):239;
% Note that the number that multiplies the planning time is calculated by
% dividing the total time of the trials by the planning time i.e.:
% time(end) / planning time

interp_q_des = interp1(time,q_des,interp_time);

%% Plotting to Check

figure(1)
hold on
plot(time,q_des)
plot(interp_time,interp_q_des,'or')

filename = 'DebugHardwareVideo.gif';
start_idx = 5000;
for i = start_idx:5800 % length(interp_q_des)
    h = figure(2);
    if ~isnan(interp_q_des(i,:))
        show(robot,interp_q_des(i,:)')
    end
    view(90,20)
    make_animation(h,i-start_idx+1,filename)
end

%% Saving Data

% python expects nx7 array
% filename = 'HardwareVideo_MultipleTrials_06_15_2023_ROSData_q_des.csv';
% writematrix(interp_q_des,filename,'Delimiter',',');

function make_animation( h,index,filename )
    drawnow
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if index == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.02);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.02);
    end
end