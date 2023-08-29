%% ARMTD-Force Post Simulation Plotting
% Zachary Brei
% 05/25/2022
% Version 1.0

%% Loading Data

% clear all; close all; clc;

load('trial_scene_010_003.csv.mat')

%% Plotting flags

plot_states = false;
plot_control = false;
plot_accel = true;
plot_force = true;
plot_moment = true;
plot_constraint = true;
plot_torque = false;
plot_torque_control = false;
plot_torque_accel = false;
plot_zmp = false;
plot_friction_cone = false;
plot_friction_cone_2D = true;

pz_approx = false;

plot_force_approx = false;
plot_constraint_approx = false;
plot_uc_constraint_approx = false;

plot_force_approx_v2 = false;
plot_constraint_approx_v2 = false;
plot_uc_constraint_approx_v2 = false;

plot_k = false;

%% Defining Transmission Inertia
transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class

dt = 1/P.jrs_info.n_t; % note this might be wrong for some of the experiments and should be 1/128

%% Extracting Info From CUDA Code

% wrench reachable set center and radii
CUDA_f_center = [];
CUDA_n_center = [];
CUDA_f_radii = [];
CUDA_n_radii = [];
% contact constraint radii
CUDA_sep_bounds = [];
CUDA_slip_bounds = [];
CUDA_tip_bounds = [];
nan_counter = 0;
for i = 1:length(P.info.contact_constraint_radii)
    % need to check if breaking maneuver is performed
    if isnan(P.info.k_opt{1,i})
        if nan_counter < 1 % first brake
            % wrench reachable set center and radii
            CUDA_f_center = [CUDA_f_center; P.info.wrench_radii{1,i-1}(51:100,1:3)];
            CUDA_n_center = [CUDA_n_center; P.info.wrench_radii{1,i-1}(51:100,4:6)];
            CUDA_f_radii = [CUDA_f_radii; P.info.wrench_radii{1,i-1}(51:100,7:9)];
            CUDA_n_radii = [CUDA_n_radii; P.info.wrench_radii{1,i-1}(51:100,10:12)];
            % contact constraint radii
            CUDA_sep_bounds = [CUDA_sep_bounds; P.info.contact_constraint_radii{1,i-1}(51:100,:)];
            CUDA_slip_bounds = [CUDA_slip_bounds; P.info.contact_constraint_radii{1,i-1}(151:200,:)];
            CUDA_tip_bounds = [CUDA_tip_bounds; P.info.contact_constraint_radii{1,i-1}(251:300,:)];
            nan_counter = nan_counter + 1;
        else % more than one brake in a row
            % wrench reachable set center and radii
            CUDA_f_center = [CUDA_f_center; P.info.wrench_radii{1,i}(1:50,1:3)];
            CUDA_n_center = [CUDA_n_center; P.info.wrench_radii{1,i}(1:50,4:6)];
            CUDA_f_radii = [CUDA_f_radii; P.info.wrench_radii{1,i}(1:50,7:9)];
            CUDA_n_radii = [CUDA_n_radii; P.info.wrench_radii{1,i}(1:50,10:12)];
            % contact constraint radii
            CUDA_sep_bounds = [CUDA_sep_bounds; P.info.contact_constraint_radii{1,i}(1:50,:)];
            CUDA_slip_bounds = [CUDA_slip_bounds; P.info.contact_constraint_radii{1,i}(101:150,:)];
            CUDA_tip_bounds = [CUDA_tip_bounds; P.info.contact_constraint_radii{1,i}(201:250,:)];
            nan_counter = nan_counter + 1;
        end
    else % normal iteration
        % wrench reachable set center and radii
        CUDA_f_center = [CUDA_f_center; P.info.wrench_radii{1,i}(1:50,1:3)];
        CUDA_n_center = [CUDA_n_center; P.info.wrench_radii{1,i}(1:50,4:6)];
        CUDA_f_radii = [CUDA_f_radii; P.info.wrench_radii{1,i}(1:50,7:9)];
        CUDA_n_radii = [CUDA_n_radii; P.info.wrench_radii{1,i}(1:50,10:12)];
        % contact constraint radii
        CUDA_sep_bounds = [CUDA_sep_bounds; P.info.contact_constraint_radii{1,i}(1:50,:)];
        CUDA_slip_bounds = [CUDA_slip_bounds; P.info.contact_constraint_radii{1,i}(101:150,:)];
        CUDA_tip_bounds = [CUDA_tip_bounds; P.info.contact_constraint_radii{1,i}(201:250,:)];
        nan_counter = 0;
    end
end

%% Create Zonotopes of Wrenches

for i = 1:size(CUDA_f_radii,1)
    CUDA_f_zono{i} = zonotope(CUDA_f_center(i,:)',diag(CUDA_f_radii(i,:)));
    CUDA_n_zono{i} = zonotope(CUDA_n_center(i,:)',diag(CUDA_n_radii(i,:)));
end

% figure(201)
% zono1 = CUDA_f_zono{1}
% plot(zono1,[1,2])

%% flags

plot_states = true;
plot_control = true;
plot_accel = true;
plot_force = true;
plot_moment = true;
plot_constraint = true;
plot_torque = true;
plot_torque_control = false;
plot_torque_accel = false;
plot_zmp = true;
plot_friction_cone = false;

pz_approx = false;

plot_force_approx = false;
plot_constraint_approx = false;
plot_uc_constraint_approx = false;

plot_force_approx_v2 = false;
plot_constraint_approx_v2 = false;
plot_uc_constraint_approx_v2 = false;

plot_k = false;

% for i = 1:length(A.time)
%     figure(101)
%     subplot(3,3,[1 2 4 5])
%     view(3)
%     show(robot,A.state(A.joint_state_indices,i))
%     subplot(3,3,3)
%     plot(A.time(1:i),A.state(A.joint_state_indices,1:i))
%     subplot(3,3,6)
%     plot(A.time(1:i),A.state(A.joint_speed_indices,1:i))
% end

%% plotting states

joint_angles = A.state(A.joint_state_indices,:);
joint_angular_velocity = A.state(A.joint_speed_indices,:);

if plot_states
    
    figure(2)
    subplot(3,1,1)
    % plotting angles of joints
    hold on
    plot(A.time,joint_angles(1,:))
    plot(A.time,joint_angles(2,:))
    plot(A.time,joint_angles(3,:))
    plot(A.time,joint_angles(4,:))
    plot(A.time,joint_angles(5,:))
    plot(A.time,joint_angles(6,:))
    plot(A.time,joint_angles(7,:))
    legend('Shoulder Pan Joint','Shoulder Lift Joint','Upper Arm Roll Joint','Elbow Flex Joint','Forearm Roll Joint','Wrist Flex Joint','Wrist Roll Joint')
    subplot(3,1,2)
    % plotting angular velocities of joints
    hold on
    plot(A.time,joint_angular_velocity(1,:))
    plot(A.time,joint_angular_velocity(2,:))
    plot(A.time,joint_angular_velocity(3,:))
    plot(A.time,joint_angular_velocity(4,:))
    plot(A.time,joint_angular_velocity(5,:))
    plot(A.time,joint_angular_velocity(6,:))
    plot(A.time,joint_angular_velocity(7,:))
    legend('Shoulder Pan Joint','Shoulder Lift Joint','Upper Arm Roll Joint','Elbow Flex Joint','Forearm Roll Joint','Wrist Flex Joint','Wrist Roll Joint')

end

%% plotting acceleration

% plotting angular accelerations of joints
% accel_short = [zeros(7,1) A.acceleration]; % (:,1:length(A.time)); % shouldn't need this anymore

if plot_accel
    
%     subplot(3,1,3)
%     hold on
%     plot(A.time,accel_short(1,:))
%     plot(A.time,accel_short(2,:))
%     plot(A.time,accel_short(3,:))
%     plot(A.time,accel_short(4,:))
%     plot(A.time,accel_short(5,:))
%     plot(A.time,accel_short(6,:))
%     plot(A.time,accel_short(7,:))
%     legend('Shoulder Pan Joint','Shoulder Lift Joint','Upper Arm Roll Joint','Elbow Flex Joint','Forearm Roll Joint','Wrist Flex Joint','Wrist Roll Joint')

    qdd_post = zeros(7,length(A.time));
    % calculating the acceleration in post to compare with what is stored
    for i = 1:length(A.time(1:end-1)) % parfor this
        [M, C, g] = A.calculate_dynamics(joint_angles(:,i), joint_angular_velocity(:,i), A.params.true);

        %% can I call u=A.LLC.get_control_inputs() here with the P.info?

        for j = 1:size(M,1)
            M(j,j) = M(j,j) + transmision_inertia(j);
        end
        
        % need to initialize this with a column of zeros and start at the
        % second index - fixed
        qdd_post(:,i) = M\(A.input(:,i+1)-C*joint_angular_velocity(:,i)-g);
%         qdd_post(:,i) = M\(A.input(:,i)-C*joint_angular_velocity(:,i)-g);

    end

    figure(5)
    subplot(2,1,1)
    hold on
%     plot(A.time,accel_short(1,:))
%     plot(A.time,accel_short(2,:))
%     plot(A.time,accel_short(3,:))
%     plot(A.time,accel_short(4,:))
%     plot(A.time,accel_short(5,:))
%     plot(A.time,accel_short(6,:))
%     plot(A.time,accel_short(7,:))
    legend('Shoulder Pan Joint','Shoulder Lift Joint','Upper Arm Roll Joint','Elbow Flex Joint','Forearm Roll Joint','Wrist Flex Joint','Wrist Roll Joint')

    subplot(2,1,2)
    hold on
    plot(A.time(1:end),qdd_post(1,:))
    plot(A.time(1:end),qdd_post(2,:))
    plot(A.time(1:end),qdd_post(3,:))
    plot(A.time(1:end),qdd_post(4,:))
    plot(A.time(1:end),qdd_post(5,:))
    plot(A.time(1:end),qdd_post(6,:))
    plot(A.time(1:end),qdd_post(7,:))
    legend('Shoulder Pan Joint','Shoulder Lift Joint','Upper Arm Roll Joint','Elbow Flex Joint','Forearm Roll Joint','Wrist Flex Joint','Wrist Roll Joint')
    % the spike seen at the very beginning might be due to singular
    % configuration!
    
end

%% calling rnea


for i = 1:length(A.time) % parfor this

    % clear relevant variables
    clear tau f n

    % call rnea
    [tau, f, n] = rnea(joint_angles(:,i)',joint_angular_velocity(:,i)',joint_angular_velocity(:,i)',qdd_post(:,i)',true,A.params.true); % A.acceleration(:,i)'
%     [tau, f, n] = rnea(joint_angles(:,i)',joint_angular_velocity(:,i)',joint_angular_velocity(:,i)',accel_short(:,i)',true,A.params.true); % A.acceleration(:,i)'

    % store rnea results
    Tau{i} = tau;
    F{i} = f;
    N{i} = n;

    % store the contact forces
    force(:,i) = f(:,10);
    % store the contact moments
    moment(:,i) = n(:,10);

end

%% Plotting the forces

if plot_force

    figure(3)
    % plot the x-axis force (in the tray frame)
    subplot(1,3,1)
    hold on
    plot(A.time(1:end), force(1,:), 'k')
    xlabel('time (s)')
    ylabel('x-axis Force (N)')
    axis('square')
    grid on
    % plot the y-axis force (in the tray frame)
    subplot(1,3,2)
    hold on
    plot(A.time(1:end), force(2,:), 'k')
    xlabel('time (s)')
    ylabel('y-axis Force (N)')
    axis('square')
    grid on
    % plot the z-axis force (in the tray frame)
    subplot(1,3,3)
    hold on
    plot(A.time(1:end), force(3,:), 'k')
    xlabel('time (s)')
    ylabel('z-axis Force (N)')
    axis('square')
    grid on

    %% plotting pz overapproximation

    % note: change plotting to be patches

    figure(102)
    % plot the x-axis force (in the tray frame)
    subplot(1,3,1)
    hold on
    plot(A.time(1:end), force(1,:), 'k')
    plot(A.time(1:end-1), CUDA_f_center(:,1)-CUDA_f_radii(:,1),'m')
    plot(A.time(1:end-1), CUDA_f_center(:,1)+CUDA_f_radii(:,1),'m')
    xlabel('time (s)')
    ylabel('x-axis Force (N)')
    axis('square')
    grid on
    % plot the y-axis force (in the tray frame)
    subplot(1,3,2)
    hold on
    plot(A.time(1:end), force(2,:), 'k')
    plot(A.time(1:end-1), CUDA_f_center(:,2)-CUDA_f_radii(:,2),'m')
    plot(A.time(1:end-1), CUDA_f_center(:,2)+CUDA_f_radii(:,2),'m')
    xlabel('time (s)')
    ylabel('y-axis Force (N)')
    axis('square')
    grid on
    % plot the z-axis force (in the tray frame)
    subplot(1,3,3)
    hold on
    plot(A.time(1:end), force(3,:), 'k')
    plot(A.time(1:end-1), CUDA_f_center(:,3)-CUDA_f_radii(:,3),'m')
    plot(A.time(1:end-1), CUDA_f_center(:,3)+CUDA_f_radii(:,3),'m')
    xlabel('time (s)')
    ylabel('z-axis Force (N)')
    axis('square')
    grid on

    %% plotting pz overapproximation with patches
    
    % plotting settings
    edgecolor = 'none';
    facealpha = 0.2;

    % note: change plotting to be patches

    figure(301)
    % plot the full nominal force trajectory first
    subplot(1,3,1)
    hold on
    plot(A.time(1:50), force(1,1:50), 'k')
    subplot(1,3,2)
    hold on
    plot(A.time(1:50), force(2,1:50), 'k')
    subplot(1,3,3)
    hold on
    plot(A.time(1:50), force(3,1:50), 'k')
    % get PZ points to plot
    fx_l = CUDA_f_center(:,1)-CUDA_f_radii(:,1);
    fx_u = CUDA_f_center(:,1)+CUDA_f_radii(:,1);
    fy_l = CUDA_f_center(:,2)-CUDA_f_radii(:,2);
    fy_u = CUDA_f_center(:,2)+CUDA_f_radii(:,2);
    fz_l = CUDA_f_center(:,3)-CUDA_f_radii(:,3);
    fz_u = CUDA_f_center(:,3)+CUDA_f_radii(:,3);
    % plot the PZ patches
    for i=1:length(A.time(1:50))
        % plot the x-axis force (in the tray frame)
        subplot(1,3,1)
        hold on
%         plot(A.time(1:end), force(1,:), 'k')
%         plot(A.time(i), CUDA_f_center(:,1)-CUDA_f_radii(:,1),'m')
%         plot(A.time(i), CUDA_f_center(:,1)+CUDA_f_radii(:,1),'m')
        pf1 = patch([A.time(i)+dt;A.time(i)+dt;A.time(i);A.time(i)],[fx_u(i);fx_l(i);fx_l(i);fx_u(i)],'m','EdgeColor',edgecolor,'FaceAlpha',facealpha);
        xlabel('time (s)')
        ylabel('x-axis Force (N)')
        axis('square')
        grid on
        % plot the y-axis force (in the tray frame)
        subplot(1,3,2)
        hold on
%         plot(A.time(1:end), force(2,:), 'k')
%         plot(A.time(1:end-1), CUDA_f_center(:,2)-CUDA_f_radii(:,2),'m')
%         plot(A.time(1:end-1), CUDA_f_center(:,2)+CUDA_f_radii(:,2),'m')
        pf2 = patch([A.time(i)+dt;A.time(i)+dt;A.time(i);A.time(i)],[fy_u(i);fy_l(i);fy_l(i);fy_u(i)],'m','EdgeColor',edgecolor,'FaceAlpha',facealpha);
        xlabel('time (s)')
        ylabel('y-axis Force (N)')
        axis('square')
        grid on
        % plot the z-axis force (in the tray frame)
        subplot(1,3,3)
        hold on
%         plot(A.time(1:end), force(3,:), 'k')
%         plot(A.time(1:end-1), CUDA_f_center(:,3)-CUDA_f_radii(:,3),'m')
%         plot(A.time(1:end-1), CUDA_f_center(:,3)+CUDA_f_radii(:,3),'m')
        pf2 = patch([A.time(i)+dt;A.time(i)+dt;A.time(i);A.time(i)],[fz_u(i);fz_l(i);fz_l(i);fz_u(i)],'m','EdgeColor',edgecolor,'FaceAlpha',facealpha);
        xlabel('time (s)')
        ylabel('z-axis Force (N)')
        axis('square')
        grid on
    end
    
end



%% Calculating Constraints

% separation constraint
sep = -1*force(3,:);

% slipping constraint
slip = sqrt(force(1,:).^2+force(2,:).^2) - W.u_s.*abs(force(3,:));
slip2 = force(1,:).^2+force(2,:).^2 - W.u_s^2.*force(3,:).^2;

for i = 1:length(A.time)
    % tipping constraint the normal way
    ZMP_top = cross([0;0;1],moment(:,i)); % normal vector should come first
    ZMP_bottom = dot([0;0;1],force(:,i));
    ZMP(:,i) = ZMP_top/ZMP_bottom;
    ZMP_rad(i) = sqrt(ZMP(1,i)^2+ZMP(2,i)^2);
    tip(i) = ZMP_rad(i) - W.surf_rad;
    % tipping constraint the PZ way
    ZMP_top2 = cross([0;0;1],moment(:,i));
    ZMP_bottom2 = dot([0;0;1],force(:,i));
    tip2(i) = ZMP_top(1)^2 + ZMP_top(2)^2 - ZMP_bottom^2*(W.surf_rad)^2;
end
%% Calculating PZ Constraints

for i = 1:length(A.time(1:end-1)) %end-1))
    f_PZ{i} = polyZonotope_ROAHM(CUDA_f_zono{i}.Z(:,1),CUDA_f_zono{i}.Z(:,2:4));
    n_PZ = polyZonotope_ROAHM(CUDA_n_zono{i}.Z(:,1),CUDA_n_zono{i}.Z(:,2:4));
    % calculating the PZ form of the constraints
    ZMP_PZ_top = cross([0;0;1],n_PZ);
    ZMP_PZ_bottom = f_PZ{i}*[0,0,1];
    ZMP_PZ_bottom = interval(ZMP_PZ_bottom);
    ZMP_PZ_bottom_inf = ZMP_PZ_bottom.inf;
    ZMP_PZ_bottom_sup = ZMP_PZ_bottom.sup;
    % Note that this is not the entire overapproximated PZ because the
    % bottom has the .sup as well
    ZMP_PZ_inf = interval(ZMP_PZ_top) / ZMP_PZ_bottom_inf;
    ZMP_PZ_sup = interval(ZMP_PZ_top) / ZMP_PZ_bottom_sup;
    ZMP_PZ{i} = convHull(ZMP_PZ_inf,ZMP_PZ_sup);
end

%% Plotting the ZMP Point

figure(401)
% subplot(3,4,12)
hold on

% plot ZMP PZ overapproximation
for i = 1:length(ZMP_PZ)
    s = plot(ZMP_PZ{i},[1,2],'Filled',true,'EdgeColor','m','FaceColor','m','FaceAlpha',0.01,'EdgeAlpha',0.2);
%     alpha(s,0.2)
end

r=W.surf_rad;
x=0;
y=0;
th = linspace(0,2*pi,500);
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
tipplot1 = plot(xunit, yunit,'-r');
xlabel('x position (m)')
ylabel('y position (m)')
axis('square')
% axis equal
grid on
title('ZMP Position')

% plot nominal ZMP location
tipplot2 = plot(ZMP(1,1:50),ZMP(2,1:50),'xk')

%% plotting constraints

if plot_constraint
    
    figure(4)
    % plot the separation constraint
    subplot(1,3,1)
    hold on
    plot(A.time(1:end),sep, 'k')
    xlabel('time (s)')
    ylabel('Separation Constraint')
    axis('square')
    grid on
    % plot the slipping constraint
    subplot(1,3,2)
    hold on
    plot(A.time(1:end),slip, 'k')
    xlabel('time (s)')
    ylabel('Slipping Constraint')
    axis('square')
    grid on
    % plot the tipping constraint
    subplot(1,3,3)
    hold on
    plot(A.time(1:end),tip, 'k')
    xlabel('time (s)')
    ylabel('Tipping Constraint')
    axis('square')
    grid on


    %% plotting with PZ overapprox

    % note: change plotting of PZs to be patches

    figure(101)
    % plot the separation constraint
    subplot(1,3,1)
    hold on
    plot(A.time(1:end),sep, 'k')
    plot(A.time(1:end-1),CUDA_sep_bounds,'m')
    xlabel('time (s)')
    ylabel('Separation Constraint')
    axis('square')
    grid on

    % plot the slipping constraint
    subplot(1,3,2)
    hold on
    plot(A.time(1:end),slip2, 'k')
    plot(A.time(1:end-1),CUDA_slip_bounds,'m')
    xlabel('time (s)')
    ylabel('Slipping Constraint')
    axis('square')
    grid on

    % plot the tipping constraint
    subplot(1,3,3)
    hold on
    plot(A.time(1:end),tip2, 'k')
    plot(A.time(1:end-1),CUDA_tip_bounds,'m')
    xlabel('time (s)')
    ylabel('Tipping Constraint')
    axis('square')
    grid on

end

%% Plotting Moments on Contact Joint

if plot_moment
    
    figure(6)
    % plot the x-axis moment (in the tray frame)
    subplot(1,3,1)
    hold on
    plot(A.time(1:end), moment(1,:), 'k')
    xlabel('time (s)')
    ylabel('x-axis Moment (Nm)')
    axis('square')
    grid on
    % plot the y-axis moment (in the tray frame)
    subplot(1,3,2)
    hold on
    plot(A.time(1:end), moment(2,:), 'k')
    xlabel('time (s)')
    ylabel('y-axis Moment (Nm)')
    axis('square')
    grid on
    % plot the z-axis moment (in the tray frame)
    subplot(1,3,3)
    hold on
    plot(A.time(1:end), moment(3,:), 'k')
    xlabel('time (s)')
    ylabel('z-axis Moment (Nm)')
    axis('square')
    grid on

end

%% Plotting Joint Torques

if plot_torque
    
    figure(7)
    plot(A.time,summary.control_input)
    legend('Shoulder Pan Joint','Shoulder Lift Joint','Upper Arm Roll Joint','Elbow Flex Joint','Forearm Roll Joint','Wrist Flex Joint','Wrist Roll Joint')
    xlabel('time (s)')
    ylabel('Control Inputs (Joint Torques) (Nm)')
    axis('square')
    grid on
    
end

%% Plotting Joint Torques and Joint Accelerations

if plot_torque_accel
    
    figure(8)
    subplot(2,1,1)
    plot(A.time,summary.control_input)
    legend('Shoulder Pan Joint','Shoulder Lift Joint','Upper Arm Roll Joint','Elbow Flex Joint','Forearm Roll Joint','Wrist Flex Joint','Wrist Roll Joint')
    xlabel('time (s)')
    ylabel('Control Inputs (Joint Torques) (Nm)')
    grid on

    subplot(2,1,2)
    hold on
%     plot(A.time,accel_short(1,:))
%     plot(A.time,accel_short(2,:))
%     plot(A.time,accel_short(3,:))
%     plot(A.time,accel_short(4,:))
%     plot(A.time,accel_short(5,:))
%     plot(A.time,accel_short(6,:))
%     plot(A.time,accel_short(7,:))
    plot(A.time,qdd_post(1,:))
    plot(A.time,qdd_post(2,:))
    plot(A.time,qdd_post(3,:))
    plot(A.time,qdd_post(4,:))
    plot(A.time,qdd_post(5,:))
    plot(A.time,qdd_post(6,:))
    plot(A.time,qdd_post(7,:))
    legend('Shoulder Pan Joint','Shoulder Lift Joint','Upper Arm Roll Joint','Elbow Flex Joint','Forearm Roll Joint','Wrist Flex Joint','Wrist Roll Joint')
    xlabel('time (s)')
    ylabel('Joint Accelerations (m/s^2)')
    grid on
    
end

%% plotting torque and tau calucalted by RNEA (these should match)

%% Plotting Location of ZMP Point in the Contact Area

if plot_zmp
    
    figure(9)
    hold on
    r=W.surf_rad;
    x=0;
    y=0;
    th = linspace(0,2*pi,500);
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit,'r');
    xlabel('x position (m)')
    ylabel('y position (m)')
    axis('square')
    grid on
    title('ZMP Position in Contact Area')

    plot(ZMP(1,:),ZMP(2,:),'k','MarkerSize',2)
    
end

%% Plotting Friction Cone

if plot_friction_cone
    
    figure(10)

    r = linspace(0,5,10);
    theta = linspace(0,2*pi,50);
    [R,Theta] = meshgrid(r,theta);
    X = R.*cos(Theta);
    Y = R.*sin(Theta);
    Z = 0.5.*R; % A.u_s.*R;
    subplot(1,3,1)
    hold on
    h1 = surf(X,Y,Z,'EdgeColor','none','FaceColor','g','FaceAlpha','0.2');
    xlabel('x-axis Tangential Force (N)')
    ylabel('y-axis Tangential Force (N)')
    zlabel('z-axis Normal Force (N)')
    axis('square')
    grid on
    title('Friction Cone')

    subplot(1,3,2)
    hold on
    h1 = surf(X,Y,Z,'EdgeColor','none','FaceColor','g','FaceAlpha','0.2');
    xlabel('x-axis Tangential Force (N)')
    ylabel('y-axis Tangential Force (N)')
    zlabel('z-axis Normal Force (N)')
    axis('square')
    grid on
    title('Friction Cone')

    subplot(1,3,3)
    hold on
    h1 = surf(X,Y,Z,'EdgeColor','none','FaceColor','g','FaceAlpha','0.2');
    xlabel('x-axis Tangential Force (N)')
    ylabel('y-axis Tangential Force (N)')
    zlabel('z-axis Normal Force (N)')
    axis('square')
    grid on
    title('Friction Cone')

    % plotting friction cone ! can't do scaling individually with this !
    %% Check if this is correctly plotting the tangential vector component
    figure(10)
    origin_length = zeros(1,length(A.time(1:end)));
    subplot(1,3,1)
    h2 = quiver3(origin_length,origin_length,origin_length,force(1,:),force(2,:),force(3,:),'k');
    view(3)
    subplot(1,3,2)
    h3 = quiver3(origin_length,origin_length,origin_length,force(1,:),force(2,:),force(3,:),'k');
    view(0,0)
    subplot(1,3,3)
    h4 = quiver3(origin_length,origin_length,origin_length,force(1,:),force(2,:),force(3,:),'k');
    view(90,0)
    zlim([0 6])
    
end

if plot_friction_cone_2D

    figure(22)
    hold on
    % plot the boundary
    theta_friction = linspace(0,2*pi,100);
    r_friction = force(3,i)*W.u_s;
    fricplot4 = plot(r_friction*cos(theta_friction),r_friction*sin(theta_friction),'-r')
    % plot the nominal forces
    plot(force(1,:),force(2,:),'.k','MarkerSize',2)
    for i = 1:length(f_PZ)
%         f_int = interval(f_PZ{1});
        f = plot(f_PZ{i},[1,2],'Filled',true,'EdgeColor','m','FaceColor','m','FaceAlpha',0.01,'EdgeAlpha',0.2);
    end

    axis square
    axis equal
    grid on
    xlabel('x-axis Force (N)')
    ylabel('y-axis Force (N)')
    title('Friction Cone')

end

%%

% started plotting friction cone for gif
% for i = 1:length(A.time)
%     figure(10)
%     subplot(1,3,1)
%     h2 = quiver3(0,0,0,force(1,i),force(2,i),force(3,i),sqrt(force(1,i)^2+force(2,i)^2+force(3,i)^2),'k');
%     view(3)
%     subplot(1,3,2)
%     h2 = quiver3(0,0,0,force(1,i),force(2,i),force(3,i),sqrt(force(1,i)^2+force(2,i)^2+force(3,i)^2),'k');
%     view(0,0)
%     subplot(1,3,3)
%     h2 = quiver3(0,0,0,force(1,i),force(2,i),force(3,i),sqrt(force(1,i)^2+force(2,i)^2+force(3,i)^2),'k');
%     view(90,0)
%     zlim([0 6])
% %     pause(0.001)
%     delete(h2)
% end

% friction_cone_x = linspace(-5,5);
% friction_cone_y = linspace(-5,5);
% friction_cone_z = sqrt(friction_cone_x.^2+friction_cone_y.^2).*A.u_s;
% 
% for i = 1:length(A.time)
%     figure(10); clf;
%     figure(10)
%     subplot(1,2,1)
%     hold on
%     plot(friction_cone_x,friction_cone_z,'-k')
%     quiver(0,0,force(1,i),force(3,i),sqrt(force(1,i)^2+force(3,i)^2))
%     subplot(1,2,2)
%     hold on
%     plot(friction_cone_y,friction_cone_z,'-k')
%     quiver(0,0,force(2,i),force(3,i),sqrt(force(2,i)^2+force(3,i)^2))
% %     pause(0.001)
% end

% figure(10)
% subplot(1,2,1)
% hold on
% plot(friction_cone_x,friction_cone_z,'-k')
% quiver(zeros(1,length(A.time)),zeros(1,length(A.time)),force(1,:),force(3,:))
% subplot(1,2,2)
% hold on
% plot(friction_cone_y,friction_cone_z,'-k')
% quiver(zeros(1,length(A.time)),zeros(1,length(A.time)),force(2,:),force(3,:))



% for i = 1:length(A.time)-1
% 
%     contact_poly = P.info.contact_poly_storage{i}{1};
%     collapsed_contact_poly = polyZonotope_ROAHM(contact_poly.c+sum(abs(contact_poly.Grest),2),contact_poly.G,[],contact_poly.expMat,contact_poly.id);
% 
%     figure(101)
%     hold on
%     % plot the overapproximation
%     plot(P.info.contact_poly_storage{i}{1})
%     plot(collapsed_contact_poly,[1,2],'r')
%     % calculate the magnitude of the tangential force
% %     tan_force_mag = sqrt(force(1,i)^2 + force(2,i)^2);
% %     tan_force_dir = 
%     plot(force(1,i),force(2,i),'oc')
%     % need to add plotting of the friction cone at the z-level
%     theta_friction = linspace(0,2*pi,100);
%     r_friction = force(3,i)*A.u_s;
%     plot(r_friction*cos(theta_friction),r_friction*sin(theta_friction),'-r')
%     axis('square')
%     xlabel('x-axis Force (N)')
%     ylabel('y-axis Force (N)')
%     % need to animate
% %     pause(0.1)
% %     clf(16)
% 
% end
% 
% 
% 
% %% Slicing the passed out PZs for the forces and the constraints
% 
% %%% NEED TO FIGURE OUT THE TIMING AND INDEXING !!!
% %%% A.time is 1x501, but constraints are 1x500
% 
% tt = 0.5;
% j = 1;
% 
% force_x_pz = cell(1,length(A.time));
% contact_pz_check = [];
% moment_pz_check = [];
% tau_pz_check = [];
% 
% if pz_approx
%     for i = 1:length(A.time)-1
%         
%         % need to detect a braking maneuver and if braking, the tt is added to
%         % but the k doesn't switch?
%     
%         %%% need to take the desired accel, which should be stored in the
%         %%% desired trajectory, convert that back to a k and use that k to
%         %%% slice at?
%         
%         if A.time(i) < tt
%             if isnan(P.info.k_opt{j}) % this corresponds to a braking maneuver
%                 % need to calculate the k for the braking maneuver 
%     %             k = (A.acceleration(i)-P.jrs_info.c_k)./P.jrs_info.g_k;
%                 k(:,i) = k_real_brake;
%             else
%                 k(:,i) = P.info.k_opt{j};
%             end
%         else
%     
%             tt = tt + 0.5; % adjust time to the next planning iteration
%             j = j + 1; % adjust k_opt index to get the k's for the next planning iteration
%             if isnan(P.info.k_opt{j}) % this corresponds to a braking maneuver
%                 % need to calculate the k for the braking maneuver
%     %             qd_peak = summary.trajectory(A.joint_speed_indices,i-1); % i-1
%     %             qdd_des = (0 - qd_peak)/P.t_plan;
%     %             qdd_des = (0 - qd_peak)/P.t_stop;
%     %             k_brake = (qdd_des-P.jrs_info.c_k)./P.jrs_info.g_k;
%     %             for g=1:length(k_brake)
%     %                 if k_brake(g) > 0
%     %                     k(g,i) = min(1,k_brake(g));
%     %                 else
%     %                     k(g,i) = max(-1,k_brake(g));
%     %                 end
%     %             end
%     %             k(:,i) = k_brake;
%                 k_real_brake = P.info.k_opt{j-1}; % k(:,i);
%                 k(:,i) = k_real_brake;
%     %             k
%     %             k_other = (A.acceleration(:,i+1)-P.jrs_info.c_k)./P.jrs_info.g_k
%     %             k_other2 = (A.acceleration(:,i)-P.jrs_info.c_k)./P.jrs_info.g_k
%     %             k_other3 = (qdd_post(:,i)-P.jrs_info.c_k)./P.jrs_info.g_k
%     %             k_other4 = (qdd_post(:,i+5)-P.jrs_info.c_k)./P.jrs_info.g_k
%             else
%                 k(:,i) = P.info.k_opt{j};
%             end
%         end
%     
%         % checking Fx lives inside overapproximation
%         force_x_pz{i} = getSubset(P.info.fx_storage{i}{1},P.info.fx_storage{i}{1}.id,k(P.info.fx_storage{i}{1}.id,i));
%         force_x_int{i} = interval(force_x_pz{i});
%         force_x_sup(i) = force_x_int{i}.sup;
%         force_x_inf(i) = force_x_int{i}.inf;
%         if force(1,i) < force_x_int{i}.sup && force(1,i) > force_x_int{i}.inf
%             test(1,i) = 1;
%         else
%             test(1,i) = 0;
%         end
%         
%         if plot_force_approx
%             figure(11)
%             subplot(1,3,1)
%             hold on
%             plot(A.time(i),force_x_int{i}.sup,'or')
%             plot(A.time(i),force_x_int{i}.inf,'or')
%             plot(A.time(i),force(1,i),'og')
%             axis('square')
%             grid on
%         end
%     
%         % checking Fy lives inside overapproximation
%         force_y_pz{i} = getSubset(P.info.fy_storage{i}{1},P.info.fy_storage{i}{1}.id,k(P.info.fy_storage{i}{1}.id,i));
%         force_y_int{i} = interval(force_y_pz{i});
%         force_y_sup(i) = force_y_int{i}.sup;
%         force_y_inf(i) = force_y_int{i}.inf;
%         if force(2,i) < force_y_int{i}.sup && force(2,i) > force_y_int{i}.inf
%             test(2,i) = 1;
%         else
%             test(2,i) = 0;
%         end
%         
%         if plot_force_approx
%             figure(11)
%             subplot(1,3,2)
%             hold on
%             plot(A.time(i),force_y_int{i}.sup,'or')
%             plot(A.time(i),force_y_int{i}.inf,'or')
%             plot(A.time(i),force(2,i),'og')
%             axis('square')
%             grid on
%         end
%     
%         % checking Fz lives inside overapproximation
%         force_z_pz{i} = getSubset(P.info.fz_storage{i}{1},P.info.fz_storage{i}{1}.id,k(P.info.fz_storage{i}{1}.id,i));
%         force_z_int{i} = interval(force_z_pz{i});
%         force_z_sup(i) = force_z_int{i}.sup;
%         force_z_inf(i) = force_z_int{i}.inf;
%         if force(3,i) < force_z_int{i}.sup && force(3,i) > force_z_int{i}.inf
%             test(3,i) = 1;
%         else
%             test(3,i) = 0;
%         end
%     
%         if plot_force_approx
%             figure(11)
%             subplot(1,3,3)
%             hold on
%             plot(A.time(i),force_z_int{i}.sup,'or')
%             plot(A.time(i),force_z_int{i}.inf,'or')
%             plot(A.time(i),force(3,i),'og')
%             axis('square')
%             grid on
%         end
%     
%         % ----------------------------------------------------
%     
%         % checking separation constraint value is less than sliced value of
%         % overapproximation
%         sep_constraint_pz{i} = getSubset(P.info.sep_constraint_storage{i}{1},P.info.sep_constraint_storage{i}{1}.id,k(P.info.sep_constraint_storage{i}{1}.id,i));
%         sep_constraint_int{i} = interval(sep_constraint_pz{i});
%         sep_constraint_sup(i) = sep_constraint_int{i}.sup;
%         sep_constraint_inf(i) = sep_constraint_int{i}.inf;
%         if sep(i) < sep_constraint_int{i}.sup && sep(i) < sep_constraint_int{i}.inf
%             test(4,i) = 1;
%         else
%             test(4,i) = 0;
%         end
%         
%         if plot_constraint_approx
%             figure(12)
%             subplot(1,3,1)
%             hold on
%             plot(A.time(i),sep_constraint_int{i}.sup,'or')
%             plot(A.time(i),sep_constraint_int{i}.inf,'+r')
%             plot(A.time(i),sep(i),'og')
%             axis('square')
%             grid on
%         end
%     
%         % checking separation constraint lives inside overapproximation
%         sep_constraint_uc_int{i} = interval(P.info.sep_uc_constraint_storage{i}{1});
%         sep_constraint_uc_sup(i) = sep_constraint_uc_int{i}.sup;
%         sep_constraint_uc_inf(i) = sep_constraint_uc_int{i}.inf;
%         sep(i);
%         if sep(i) < sep_constraint_uc_int{i}.sup && sep(i) > sep_constraint_uc_int{i}.inf
%             test2(1,i) = 1;
%         else
%             test2(1,i) = 0;
%         end
%     
%         if plot_uc_constraint_approx
%             figure(13)
%             subplot(1,3,1)
%             hold on
%             plot(A.time(i),sep_constraint_uc_int{i}.sup,'or')
%             plot(A.time(i),sep_constraint_uc_int{i}.inf,'+r')
%             plot(A.time(i),sep(i),'og')
%             axis('square')
%             grid on
%         end
%     
%         % checking slipping constraint value is less than sliced value of
%         % overapproximation
%     
%         %%% NOTE: the regular slipping constraint is not squared like the
%         %%% overapproximation is. so the sliced value may not always be
%         %%% overapproximative of the regular constraint. should rewrite the
%         %%% regular constraint and see.
%     
%         slip_constraint_pz{i} = getSubset(P.info.slip_constraint_storage{i}{1},P.info.slip_constraint_storage{i}{1}.id,k(P.info.slip_constraint_storage{i}{1}.id,i));
%         slip_constraint_int{i} = interval(slip_constraint_pz{i});
%         slip_constraint_sup(i) = slip_constraint_int{i}.sup;
%         slip_constraint_inf(i) = slip_constraint_int{i}.inf;
%         if slip2(i) < slip_constraint_int{i}.sup && slip2(i) < slip_constraint_int{i}.inf
%             test(5,i) = 1;
%         else
%             test(5,i) = 0;
%         end
%     
%         if plot_constraint_approx
%             figure(12)
%             subplot(1,3,2)
%             hold on
%             plot(A.time(i),slip_constraint_int.sup{i},'or')
%             plot(A.time(i),slip_constraint_int.inf{i},'+r')
%             plot(A.time(i),slip2(i),'og')
%             axis('square')
%             grid on
%         end
%     
%         % checking slipping constraint lives inside overapproximation
%         slip_constraint_uc_int{i} = interval(P.info.slip_uc_constraint_storage{i}{1});
%         slip_constraint_uc_sup(i) = slip_constraint_uc_int{i}.sup;
%         slip_constraint_uc_inf(i) = slip_constraint_uc_int{i}.inf;
%         slip2(i);
%         if slip2(i) < slip_constraint_uc_int{i}.sup && slip2(i) > slip_constraint_uc_int{i}.inf
%             test2(2,i) = 1;
%         else
%             test2(2,i) = 0;
%         end
%     
%         if plot_uc_constraint_approx
%             figure(13)
%             subplot(1,3,2)
%             hold on
%             plot(A.time(i),slip_constraint_uc_int{i}.sup,'or')
%             plot(A.time(i),slip_constraint_uc_int{i}.inf,'+r')
%             plot(A.time(i),slip(i),'og')
%             plot(A.time(i),slip2(i),'+g')
%             axis('square')
%             grid on
%         end
%     
%         % checking tipping constraint value is less than sliced value of
%         % overapproximation
%         tip_constraint_pz{i} = getSubset(P.info.tip_constraint_storage{i}{1},P.info.tip_constraint_storage{i}{1}.id,k(P.info.tip_constraint_storage{i}{1}.id,i));
%         tip_constraint_int{i} = interval(tip_constraint_pz{i});
%         tip_constraint_sup(i) = tip_constraint_int{i}.sup;
%         tip_constraint_inf(i) = tip_constraint_int{i}.inf;
%         if tip2(i) < tip_constraint_int{i}.sup && tip2(i) < tip_constraint_int{i}.inf
%             test(6,i) = 1;
%         else
%             test(6,i) = 0;
%         end
%     
%         if plot_constraint_approx
%             figure(12)
%             subplot(1,3,3)
%             hold on
%             plot(A.time(i),tip_constraint_int.sup{i},'or')
%             plot(A.time(i),tip_constraint_int.inf{i},'+r')
%             plot(A.time(i),tip(i),'og')
%             plot(A.time(i),tip2(i),'+g')
%             axis('square')
%             grid on
%         end
%     
%         % checking tipping constraint lives inside overapproximation
%         tip_constraint_uc_int{i} = interval(P.info.tip_uc_constraint_storage{i}{1});
%         tip_constraint_uc_sup(i) = tip_constraint_uc_int{i}.sup;
%         tip_constraint_uc_inf(i) = tip_constraint_uc_int{i}.inf;
%         tip2(i);
%         if tip2(i) < tip_constraint_uc_int{i}.sup && tip2(i) > tip_constraint_uc_int{i}.inf
%             test2(3,i) = 1;
%         else
%             test2(3,i) = 0;
%         end
%     
%         if plot_uc_constraint_approx
%             figure(13)
%             subplot(1,3,3)
%             hold on
%             plot(A.time(i),tip_constraint_uc_int{i}.sup,'or')
%             plot(A.time(i),tip_constraint_uc_int{i}.inf,'+r')
%             plot(A.time(i),tip(i),'og')
%             plot(A.time(i),tip2(i),'+g')
%             axis('square')
%             grid on
%         end
%     
%         % contact force polynomial check
%     
% %         % slice the contact polynomial
% %         contact_poly_sliced{i} = getSubset(P.info.contact_poly_storage{i}{1},P.info.contact_poly_storage{i}{1}.id,k(P.info.contact_poly_storage{i}{1}.id,i));
% %         % convert to a polytope using polytope_PH
% %         concat_contact_poly_sliced{i} = [contact_poly_sliced{i}.c,contact_poly_sliced{i}.Grest];
% %         [A_contact, b_contact] = polytope_PH(concat_contact_poly_sliced{i});
% %         % check if RNEA ouput lives inside the overapproximation
% %     %     test3 = A_contact*force(:,i) - b_contact
% %     %     test1 = all(A_contact*force(:,i) - b_contact <= 0, 1)
% %     %     test2 = ~(all(A_contact*force(:,i) - b_contact <= 0, 1))
% %         if ~(all(A_contact*force(:,i) - b_contact <= 0, 1))
% %             contact_pz_check(end+1) = i;
% %         end
%     
%         % contact moment polynomial check
% 
% %         moment_poly_sliced{i} = getSubset(P.info.pzrnea_moment_storage{i}{1},P.info.pzrnea_moment_storage{i}{1}.id,k(P.info.pzrnea_moment_storage{i}{1}.id,i));
% %         % convert to a polytope using polytope_PH
% %         concat_moment_poly_sliced{i} = [moment_poly_sliced{i}.c,moment_poly_sliced{i}.Grest];
% %         [A_moment, b_moment] = polytope_PH(concat_moment_poly_sliced{i});
% %         % check if RNEA ouput lives inside the overapproximation
% %     %     test3 = A_contact*force(:,i) - b_contact
% %     %     test1 = all(A_contact*force(:,i) - b_contact <= 0, 1)
% %     %     test2 = ~(all(A_contact*force(:,i) - b_contact <= 0, 1))
% %         if ~(all(A_moment*moment(:,i) - b_moment <= 0, 1))
% %             moment_pz_check(end+1) = i;
% %         end
%     
%         % pzrnea tau polynomial check
%         % need to iterate through all the joints
%     %     for j = 1:7
%     %         tau_poly_sliced{i,j} = getSubset(P.info.pzrnea_tau_storage{i}{1}{j},P.info.pzrnea_tau_storage{i}{1}{j}.id,k(P.info.pzrnea_tau_storage{i}{1}{j}.id,i));
%     %         % convert to a polytope using polytope_PH
%     %         concat_tau_poly_sliced{i,j} = [tau_poly_sliced{i,j}.c,tau_poly_sliced{i,j}.Grest];
%     %         [A_tau, b_tau] = polytope_PH(concat_tau_poly_sliced{i,j});
%     %         % check if RNEA ouput lives inside the overapproximation
%     %     %     test3 = A_contact*force(:,i) - b_contact
%     %     %     test1 = all(A_contact*force(:,i) - b_contact <= 0, 1)
%     %     %     test2 = ~(all(A_contact*force(:,i) - b_contact <= 0, 1))
%     %         if ~(all(A_tau*Tau{i,1}(j) - b_tau <= 0, 1))
%     %             tau_pz_check(j,end+1) = i;
%     %         end
%     %     end
%     
%     end
% 
%     %% plotting approximations
%     
%     % (1:end-1)
%     time_vec = A.time(1:end-1);
%     
%     if plot_force_approx_v2
%         figure(14)
%         
%         subplot(1,3,1)
%         hold on
%         plot(time_vec,force_x_sup,'--r')
%         plot(time_vec,force_x_inf,'r')
%         plot(A.time,force(1,:),'g')
%         axis('square')
%         grid on
%         
%         subplot(1,3,2)
%         hold on
%         plot(time_vec,force_y_sup,'--r')
%         plot(time_vec,force_y_inf,'r')
%         plot(A.time,force(2,:),'g')
%         axis('square')
%         grid on
%         
%         subplot(1,3,3)
%         hold on
%         plot(time_vec,force_z_sup,'--r')
%         plot(time_vec,force_z_inf,'r')
%         plot(A.time,force(3,:),'g')
%         axis('square')
%         grid on
%     end
%     
%     if plot_constraint_approx_v2
%         figure(12)
%         
%         subplot(1,3,1)
%         hold on
%         plot(time_vec,sep_constraint_sup,'--r')
%         plot(time_vec,sep_constraint_inf,'r')
%         plot(A.time,sep,'g')
%         axis('square')
%         grid on
%         
%         subplot(1,3,2)
%         hold on
%         plot(time_vec,slip_constraint_sup,'--r')
%         plot(time_vec,slip_constraint_inf,'r')
%         plot(A.time,slip,'g')
%         plot(A.time,slip2,'--g')
%         axis('square')
%         grid on
%         
%         subplot(1,3,3)
%         hold on
%         plot(time_vec,tip_constraint_sup,'--r')
%         plot(time_vec,tip_constraint_inf,'r')
%         plot(A.time,tip,'g')
%         plot(A.time,tip2,'--g')
%         axis('square')
%         grid on
%     end
%         
%     if plot_uc_constraint_approx_v2
%         figure(13)
%         
%         subplot(1,3,1)
%         hold on
%         plot(time_vec,sep_constraint_uc_sup,'--r')
%         plot(time_vec,sep_constraint_uc_inf,'r')
%         plot(A.time,sep,'g')
%         axis('square')
%         grid on
%         
%         subplot(1,3,2)
%         hold on
%         plot(time_vec,slip_constraint_uc_sup,'--r')
%         plot(time_vec,slip_constraint_uc_inf,'r')
%         plot(A.time,slip,'g')
%         plot(A.time,slip2,'--g')
%         axis('square')
%         grid on
%         
%         subplot(1,3,3)
%         hold on
%         plot(time_vec,tip_constraint_uc_sup,'--r')
%         plot(time_vec,tip_constraint_uc_inf,'r')
%         plot(A.time,tip,'g')
%         plot(A.time,tip2,'--g')
%         axis('square')
%         grid on
%     end
%     
%     if plot_k
%         figure(15)
%         plot(time_vec,k)
%     end
%     
% %     accel_k = P.jrs_info.c_k + P.jrs_info.g_k.*k;
%     
% %     figure(15)
% %     % note these may not match during braking maneuver since the braking
% %     % acceleration depends on the initial velocity as well.
% %     hold on
% %     plot(time_vec,accel_k,'--')
% %     plot(A.time,accel_short)
% 
% end
% 
% %% Inverse Dynamics 
% 
% % for i = 1:length(A.time)
% %     tau_id(:,i) = inverseDynamics(robot,joint_angles(:,i),joint_angular_velocity(:,i),accel_short(:,i));
% %     if abs(tau_id(:,i) - Tau{i}) >= 1e-7
% %         fprintf('not equal at ',num2str(i))
% %     end
% % end
% 
% %% Debugging Test
% 
% %% should plot the brake acceleration (v_peak)/t_plan to see if it matches
% accel_brake = -joint_angular_velocity(:,151)./0.5;
% % tested this and it matches what is in accel_short (what is actually
% % executed by the robot)
% 
% %% check what the inputs to the create_jrs_online are.
% 
% %% check all outputs of pzrnea against rnea
% 
% 
% %% store jrs output and see if trajectory lives inside it.
% 
% %% check storing of PZs
% 
% %% check plotting especially during braking maneuver
% 
% %% do one planning iteration and then see if I have a jump. put goal super
% % close. 
% 
% %% check plotting of actual trajectory and force since there is no kink in
% % the force
% 
% %% check storing of accel
% 
% %% plot f and M and tau for base joint since that should always kink.
% 
% 
% %% To do 07/07/2022
% 
% % store the ZMP_top and ZMP_bottom, slice both and try to divide, then plot
% 
% for i = 1:length(A.time)-1
% 
%     figure(16)
%     hold on
%     % plot the overapproximation
%     plot(P.info.contact_poly_storage{i}{1})
%     % calculate the magnitude of the tangential force
% %     tan_force_mag = sqrt(force(1,i)^2 + force(2,i)^2);
% %     tan_force_dir = 
%     plot(force(1,i),force(2,i),'oc')
%     % need to add plotting of the friction cone at the z-level
%     theta_friction = linspace(0,2*pi,100);
%     r_friction = force(3,i)*A.u_s;
%     plot(r_friction*cos(theta_friction),r_friction*sin(theta_friction),'-r')
%     axis('square')
%     xlabel('x-axis Force (N)')
%     ylabel('y-axis Force (N)')
%     % need to animate
% %     pause(0.1)
% %     clf(16)
% 
% end
% 
% 
% contact_poly = P.info.contact_poly_storage{i}{1};
% collapsed_contact_poly = polyZonotope_ROAHM(contact_poly.c+sum(abs(contact_poly.Grest)),contact_poly.G,[],contact_poly.id,contact_poly.exp_Mat);
% 
% for i = 1:length(A.time)-1
% 
%     figure()
%     hold on
%     % plot the overapproximation
%     plot(P.info.contact_poly_storage{i}{1})
%     plot(collapsed_contact_poly)
%     % calculate the magnitude of the tangential force
% %     tan_force_mag = sqrt(force(1,i)^2 + force(2,i)^2);
% %     tan_force_dir = 
%     plot(force(1,i),force(2,i),'oc')
%     % need to add plotting of the friction cone at the z-level
%     theta_friction = linspace(0,2*pi,100);
%     r_friction = force(3,i)*A.u_s;
%     plot(r_friction*cos(theta_friction),r_friction*sin(theta_friction),'-r')
%     axis('square')
%     xlabel('x-axis Force (N)')
%     ylabel('y-axis Force (N)')
%     % need to animate
% %     pause(0.1)
% %     clf(16)
% 
% end

