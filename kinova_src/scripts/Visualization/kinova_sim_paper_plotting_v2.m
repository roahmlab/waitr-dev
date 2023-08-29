clear; 
close all; 
clc;

%%

fig_num = 1;
fontsize = 16;
linewidth = 3;
% number of time steps to skip
skip_num = 5;

% planning iteration to plot
plan_index = 1;
% start index of the planning iteration?
traj_index = 1;

%% To Do

% calculate sliced ZMP overapproximation

% calculate unsliced overapproximations
    % desired trajectory
    % jrs?
    % pzrnea

% plot ZMP

% plot separation

% choose world to plot
    % choose planning iteration to plot

% format plots to be all in one subplot? have grey then?

%% Load Simulation Result

sim_result_file = 'trial_scene_010_003.csv.mat';
sim_result = load(sim_result_file);

% extract information
A = sim_result.A;
W = sim_result.W;
S = sim_result.S;
P = sim_result.P;
summary = sim_result.summary;

time_steps = P.jrs_info.n_t;
time_index = time_steps/2;
duration = P.DURATION;
dt = duration/time_steps;
t_traj = linspace(0,duration,time_steps);
surf_rad = W.surf_rad;
u_s = W.u_s;

%% Setting Up World Plotting

u_s = W.u_s;

agent_info = summary.agent_info ;
bounds = summary.bounds ;
obstacles = summary.obstacles ;
planner_info = summary.planner_info ;
start = summary.start ;
goal = summary.goal ;

% agent just for visualizing, parameters may differ
agent_urdf = 'Kinova_Grasp_Cylinder_Edge.urdf';
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot);

% create world
goal_type = 'configuration';
goal_radius = pi/30;

% fill in agent state
A.time = agent_info.time ;
A.state = agent_info.state ;
A.use_CAD_flag = true;
n_links_and_joints = params.nominal.num_joints;
n_states = 2*params.nominal.num_q;
q_index = params.nominal.q_index;

joint_state_indices = 1:2:n_states ;
joint_speed_indices = 2:2:n_states ;
joint_types = params.nominal.joint_types';
joint_axes = params.nominal.joint_axes;

link_shapes = repmat({'cuboid'}, 1, n_links_and_joints);
[link_poly_zonotopes, link_sizes, temp_link_CAD_data] = create_pz_bounding_boxes(robot);
A.load_CAD_arm_patch_data(temp_link_CAD_data)
A.link_plot_edge_color = [0 0 1] ;
A.link_plot_edge_opacity = 0 ;
A.link_plot_face_color = [0.8 0.8 1] ;
A.link_plot_face_opacity = 1 ;
% set up world using arm
W.setup(agent_info)
clear_plot_data(W);

%% Plotting World

fig_num = fig_num + 1;
sim_robot_fig = figure(fig_num);
hold on

plot(W) ;
view(3)
grid off
axis off
axis equal
plot_at_time(A, 0);
set(gcf,'color','w')
view([45,20])

exportgraphics(sim_robot_fig,'PaperSimFig_Robot_v3.jpg','Resolution',300)


% hold on

%% Color Coding

history_line_color = 1/256*[211,211,211];
unsliced_color = 1/256*[90,200,243];
% slice_color = 1/256*[72,181,163]; % light green
% slice_color = 1/256*[75,154,76]; % dark green
slice_color = 1/256*[186,85,255]; % light purple
face_alpha = 0.3;
face_alpha_light = 0.03;
edge_alpha = 0.4;

%% Calculate Nominal Constraint Values

transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; 

joint_angles = A.state(A.joint_state_indices,:);
joint_angular_velocity = A.state(A.joint_speed_indices,:);

qdd_post = zeros(7,length(A.time));
    % calculating the acceleration in post to compare with what is stored
parfor i = 1:length(A.time(1:end-1))
    [M, C, g] = A.calculate_dynamics(joint_angles(:,i), joint_angular_velocity(:,i), A.params.true);
    for j = 1:size(M,1)
        M(j,j) = M(j,j) + transmision_inertia(j);
    end
    qdd_post(:,i) = M\(A.input(:,i+1)-C*joint_angular_velocity(:,i)-g);
end

% calling rnea
Tau = cell(1,length(joint_angles));
F = cell(1,length(joint_angles));
N = cell(1,length(joint_angles));
force = zeros(3,length(joint_angles));
moment = zeros(3,length(joint_angles));
parfor i = 1:length(A.time)

    % clear relevant variables
%     clear tau f n

    % call rnea
    [tau, f, n] = rnea(joint_angles(:,i)',joint_angular_velocity(:,i)',joint_angular_velocity(:,i)',qdd_post(:,i)',true,A.params.true); % A.acceleration(:,i)'

    % store rnea results
    Tau{i} = tau;
    F{i} = f;
    N{i} = n;

    % store the contact forces
    force(:,i) = f(:,10);
    % store the contact moments
    moment(:,i) = n(:,10);

end

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

%% Calculate Unsliced Overapproximations

% get agent info
agent_info = A.get_agent_info() ;

% get world info
% given the current state of the agent, query the world
% to get the surrounding obstacles
world_info = W.get_world_info(agent_info,P) ;

% initial condition
q_0 = A.state(A.joint_state_indices,traj_index);
q_dot_0 = A.state(A.joint_speed_indices,traj_index);
q_ddot_0 = A.reference_acceleration(:,traj_index);

q_des = P.HLP.waypoints(:,plan_index);

% organize input to cuda program
P.vdisp('Calling CUDA & C++ Program!',3);
% P.kinova_test_folder_path = 'C:\Users\roahmlab\Documents\GitHub\armour-dev\kinova_src';
P.kinova_test_folder_path = '/home/baiyuew/ROAHM/armour-dev/kinova_src';
% cuda_input_file = fopen([P.kinova_test_folder_path, '\kinova_simulator_interfaces\kinova_planner_realtime\buffer\armour.in'], 'w');  
cuda_input_file = fopen([P.kinova_test_folder_path, '/kinova_simulator_interfaces/kinova_planner_realtime/buffer/armour.in'], 'w');  

for ind = 1:length(q_0)
    fprintf(cuda_input_file, '%.10f ', q_0(ind));
end
fprintf(cuda_input_file, '\n');
for ind = 1:length(q_dot_0)
    fprintf(cuda_input_file, '%.10f ', q_dot_0(ind));
end
fprintf(cuda_input_file, '\n');
for ind = 1:length(q_ddot_0)
    fprintf(cuda_input_file, '%.10f ', q_ddot_0(ind));
end
fprintf(cuda_input_file, '\n');
for ind = 1:length(q_des)
    fprintf(cuda_input_file, '%.10f ', q_des(ind));
end
fprintf(cuda_input_file, '\n');
fprintf(cuda_input_file, '%d\n', max(length(world_info.obstacles), 0));
for obs_ind = 1:length(world_info.obstacles)
    temp = reshape(world_info.obstacles{obs_ind}.Z, [1,size(world_info.obstacles{obs_ind}.Z,1) * size(world_info.obstacles{obs_ind}.Z,2)]);
    for ind = 1:length(temp)
        fprintf(cuda_input_file, '%.10f ', temp(ind));
    end
    fprintf(cuda_input_file, '\n');
end

fclose(cuda_input_file);

% call cuda program in terminal
% you have to be in the proper path!
terminal_output = system('env -i bash -i -c "../kinova_simulator_interfaces/kinova_planner_realtime/rtd_force_main_v2" '); % rtd-force path
%                     terminal_output = system('env -i bash -i -c "./../kinova_simulator_interfaces/kinova_planner_realtime_original/armour_main"'); % armour path

if terminal_output == 0
    data = readmatrix([P.kinova_test_folder_path, '/kinova_simulator_interfaces/kinova_planner_realtime/buffer/armour.out'], 'FileType', 'text');
%                         data = readmatrix([P.kinova_test_folder_path, '/kinova_simulator_interfaces/kinova_planner_realtime_original/buffer/armour.out'], 'FileType', 'text');
    k_opt = data(1:end-1);
    planning_time = data(end) / 1000.0; % original data is milliseconds

    if length(k_opt) == 1
        P.vdisp('Unable to find new trajectory!',3)
        k_opt = nan;
    elseif planning_time > P.t_plan
        P.vdisp('Solver Took Too Long!',3)
        k_opt = nan;
    else
        P.vdisp('New trajectory found!',3);
        for i = 1:length(k_opt)
            fprintf('%7.6f ', k_opt(i));
        end
        fprintf('\n');
    end
else
    error('CUDA program error! Check the executable path in armour-dev/kinova_src/kinova_simulator_interfaces/uarmtd_planner');
end

if terminal_output == 0
    % read FRS information if needed
    link_frs_center = readmatrix('armour_joint_position_center.out', 'FileType', 'text');
    link_frs_generators = readmatrix('armour_joint_position_radius.out', 'FileType', 'text');
    control_input_radius = readmatrix('armour_control_input_radius.out', 'FileType', 'text');
    constraints_value = readmatrix('armour_constraints.out', 'FileType', 'text');
    contact_constraint_radii = readmatrix('armour_force_constraint_radius.out', 'FileType', 'text');
    wrench_radii = readmatrix('armour_wrench_values.out', 'FileType', 'text');
else
    k_opt = nan;
end

% parse the unsliced PZ output
[force_PZ_unsliced,moment_PZ_unsliced] = parse_unsliced_PZs_func();

%% Get PZ Wrench Info

force_vertices = [];
for i = 1 % :length(sim_result.P.info.wrench_radii)

    wrench = sim_result.P.info.wrench_radii{i};
    
    % get center and radii values of sliced PZs
    force_center = wrench(:,1:3);
    moment_center = wrench(:,4:6);
    force_radii = wrench(:,7:9);
    moment_radii = wrench(:,10:12);

end

% getting upper and lower bounds for patch plotting
force_lower = force_center - force_radii;
force_upper = force_center + force_radii;

%% Calculate ZMP Overapproximation 

% unsliced
for i = 1:time_index
    if size(moment_PZ_unsliced{i}.Z,2) == 2
        % add two columns of zeros
        moment_zono = polyZonotope_ROAHM(moment_PZ_unsliced{i}.Z(:,1),[moment_PZ_unsliced{i}.Z(:,2), zeros(3,2)],[]);
    elseif size(moment_PZ_unsliced{i}.Z,2) == 3
        % add one column of zeros
        moment_zono = polyZonotope_ROAHM(moment_PZ_unsliced{i}.Z(:,1),[moment_PZ_unsliced{i}.Z(:,2), moment_PZ_unsliced{i}.Z(:,3), zeros(3,1)],[]);
    else
        moment_zono = polyZonotope_ROAHM(moment_PZ_unsliced{i}.Z(:,1),moment_PZ_unsliced{i}.Z(:,2:end),[]);
    end
    % calculating the PZ form of the constraints
    ZMP_PZ_top = cross([0;0;1],moment_zono);
    ZMP_PZ_bottom = interval(force_PZ_unsliced{i}); % *[0,0,1]; % modified
%     ZMP_PZ_bottom = interval(ZMP_PZ_bottom);
    ZMP_PZ_bottom_inf = ZMP_PZ_bottom.inf(3);
    ZMP_PZ_bottom_sup = ZMP_PZ_bottom.sup(3);
    % Note that this is not the entire overapproximated PZ because the
    % bottom has the .sup as well
    ZMP_PZ_inf = interval(ZMP_PZ_top) / ZMP_PZ_bottom_inf;
    ZMP_PZ_sup = interval(ZMP_PZ_top) / ZMP_PZ_bottom_sup;
    ZMP_unsliced_PZ{i} = convHull(ZMP_PZ_inf,ZMP_PZ_sup);
end

% sliced
for i = 1:time_index
    force_int{i} = interval(force_center(i,3)-force_radii(i,3),force_center(i,3)+force_radii(i,3));
    moment_zono = polyZonotope_ROAHM(moment_center(i,:)',eye(3).*moment_radii(i,:)');
    % calculating the PZ form of the constraints
    ZMP_PZ_top = cross([0;0;1],moment_zono);
    ZMP_PZ_bottom = force_int{i}; % *[0,0,1]; % modified
    ZMP_PZ_bottom_inf = ZMP_PZ_bottom.inf;
    ZMP_PZ_bottom_sup = ZMP_PZ_bottom.sup;
    % Note that this is not the entire overapproximated PZ because the
    % bottom has the .sup as well
    ZMP_PZ_inf = interval(ZMP_PZ_top) / ZMP_PZ_bottom_inf;
    ZMP_PZ_sup = interval(ZMP_PZ_top) / ZMP_PZ_bottom_sup;
    ZMP_PZ{i} = convHull(ZMP_PZ_inf,ZMP_PZ_sup);
end

%% Plotting Separation

fig_num = fig_num + 1;
figure(fig_num); clf; hold on;

% plot unsliced force overapproximation
for i = 1:time_index

    force_int_unsliced = interval(force_PZ_unsliced{i});

    fz1 = patch([t_traj(i)+dt; t_traj(i)+dt; t_traj(i); t_traj(i)], [force_int_unsliced.sup(3); force_int_unsliced.inf(3); force_int_unsliced.inf(3); force_int_unsliced.sup(3)],'b');
    fz1.LineWidth = 0.01;
    fz1.FaceColor = unsliced_color;
    fz1.FaceAlpha = face_alpha;
    fz1.EdgeAlpha = edge_alpha;

end

% plot sliced force overapproximation
for i = 1:time_index
    fz2 = patch([t_traj(i)+dt; t_traj(i)+dt; t_traj(i); t_traj(i)], [force_upper(i,3); force_lower(i,3); force_lower(i,3); force_upper(i,3)],'b');
    fz2.LineWidth = 0.01;
    fz2.FaceColor = slice_color;
    fz2.FaceAlpha = face_alpha;
    fz2.EdgeAlpha = edge_alpha;
end

% plot the nominal values
plot(t_traj(1:time_index+1), force(3,1:time_index+1),'-k','LineWidth',linewidth)

% formatting for plot
ylim([0 2.5])
xlim([0 t_traj(time_index+1)])
xlabel('Time (s)')
ylabel('z_o Force (N)')
title('Contact Joint z-Axis Force')
set(gca,'FontSize',fontsize)
set(gcf,'color','w')
box on



%% Plotting Friction Cone Diagram

fig_num = fig_num + 1;
figure(fig_num)
hold on

% plotting friction cone boundary
max_fz = max(force(3,1:time_index));
min_fz = min(force(3,1:time_index));
% [max_fz_x, max_fz_y] = circle(2*max_fz*W.u_s,0,0,0,2*pi,0.01);
% [min_fz_x, min_fz_y] = circle(2*min_fz*W.u_s,0,0,0,2*pi,0.01);
% plot(max_fz_x, max_fz_y,'-r')
% plot(min_fz_x, min_fz_y,'-r')
% fill([max_fz_x flip(min_fz_x)],[max_fz_y flip(min_fz_y)],'r','FaceAlpha',0.9)
% fp_bound = line([max_fz_x(end) min_fz_x(1)],[max_fz_y(end) min_fz_y(1)]);
% set(fp_bound,'Color','r') % ,'EdgeAlpha',0.3)

plotConcentricCircles(fig_num,min_fz*u_s,max_fz*u_s,[1,0,0,0.25],'r')

% plotting unsliced overapproximation
for i = 1:skip_num:time_index

    force_int_unsliced = interval(force_PZ_unsliced{i});

    patch([force_int_unsliced.inf(1),force_int_unsliced.sup(1),force_int_unsliced.sup(1),force_int_unsliced.inf(1)],[force_int_unsliced.inf(2),force_int_unsliced.inf(2),force_int_unsliced.sup(2),force_int_unsliced.sup(2)],unsliced_color,'FaceAlpha',face_alpha_light,'EdgeAlpha',edge_alpha,'EdgeColor',unsliced_color)

end

% plotting sliced overapproximation
for i = 1:skip_num:time_index

    patch([force_lower(i,1),force_upper(i,1),force_upper(i,1),force_lower(i,1)],[force_lower(i,2),force_lower(i,2),force_upper(i,2),force_upper(i,2)],slice_color,'FaceAlpha',face_alpha_light,'EdgeAlpha',edge_alpha,'EdgeColor',slice_color)

end

% plotting nominal values
plot(force(1,1:time_index),force(2,1:time_index),'-k')

% plot formatting
xlabel('x_o Force (N)')
ylabel('y_o Force (N)')
title('Friction Cone')
set(gca,'FontSize',fontsize)
axis square
axis equal
set(gcf,'color','w')
box on


%% Plotting ZMP Diagram

fig_num = fig_num + 1;
figure(fig_num)
hold on

% scale from meter to centimeter
factor = 100; 

% plot constraint boundary
r=surf_rad*factor;
x=0;
y=0;
th = linspace(0,2*pi,500);
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
tipplot1 = plot(xunit, yunit,'-r');


% plot unsliced
for i = 1:skip_num:time_index
    zmp1 = plot(ZMP_unsliced_PZ{i}.*factor,[1,2],'Filled',true);
    zmp1.LineWidth = 0.1;
    zmp1.FaceColor = unsliced_color;
    zmp1.EdgeColor = unsliced_color;
    zmp1.EdgeAlpha = edge_alpha;
    zmp1.FaceAlpha = face_alpha_light;
end

% plot sliced
for i = 1:skip_num:time_index
    zmp2 = plot(ZMP_PZ{i}.*factor,[1,2],'Filled',true);
    zmp2.LineWidth = 0.1;
    zmp2.FaceColor = slice_color;
    zmp2.EdgeColor = slice_color;
    zmp2.EdgeAlpha = edge_alpha;
    zmp2.FaceAlpha = face_alpha_light;
end

% plot nominal
plot(ZMP(1,1:time_index).*factor,ZMP(2,1:time_index).*factor,'-k')

% plot formatting
xlabel('x_o position (cm)')
ylabel('y_o position (cm)')
title('Zero Moment Point')
axis('square')
axis equal
% grid on
set(gca,'FontSize',fontsize)
set(gcf,'color','w')
box on


%% helper functions
function [link_poly_zonotopes, link_sizes, mesh] = create_link_poly_zonos(robot)
% takes in a robot
% creates overapproximating PZ bounding box for each body of the robot.

    for i = 1:robot.NumBodies
        bounds = zeros(3, 2);
      
        if strcmp(robot.Bodies{i}.Visuals{1}, 'primitive:box')
            col_str = robot.Bodies{i}.Collisions{1};
        
            col_str = split(col_str(11:end-1));
            
            link_sizes(:,i) = zeros(length(col_str),1);
        
            for j=1:length(col_str)
                link_sizes(j,i) = str2num(col_str{j});
            end

            c = link_sizes(:,i)/2;
            c(2) = 0;
            Grest = diag(link_sizes(:,i)/2);
    
            link_poly_zonotopes{i,1} = polyZonotope_ROAHM(c, [], Grest);
            
    
            mesh = [];
    
        elseif contains(robot.Bodies{i}.Visuals{1}, '.stl') || contains(robot.Bodies{i}.Visuals{1}, '.STL')
            stl_file = robot.Bodies{i}.Visuals{1};
            stl_file = extractAfter(stl_file, 'Mesh:');
            mesh{i, 1} = stlread(stl_file);
            
            for j = 1:3
                bounds(j, 1) = min(mesh{i}.Points(:, j));
                bounds(j, 2) = max(mesh{i}.Points(:, j));
            end
    
            c = (bounds(:, 1) + bounds(:, 2))./2;
            g = (bounds(:, 2) - bounds(:, 1))./2;
        
            centers(i,:) = c';
            
            link_poly_zonotopes{i, 1} = polyZonotope_ROAHM(c, [], [g(1), 0, 0;...
                                                                   0, g(2), 0;...
                                                                   0, 0, g(3)]);
                                                        
            link_sizes(:, i) = [2*g(1); 2*g(2); 2*g(3)];
        else
            % 10 cm cube if STL file not available
            bounds = [-0.05, 0.05; 
                      -0.05, 0.05; 
                      -0.05, 0.05];
    
    
            c = (bounds(:, 1) + bounds(:, 2))./2;
            g = (bounds(:, 2) - bounds(:, 1))./2;
        
            centers(i,:) = c';
            
            link_poly_zonotopes{i, 1} = polyZonotope_ROAHM(c, [], [g(1), 0, 0;...
                                                                   0, g(2), 0;...
                                                                   0, 0, g(3)]);
                                                        
            link_sizes(:, i) = [2*g(1); 2*g(2); 2*g(3)];
        end
    end
end

function [xlinks, ylinks] = link_points(robot, q)

    num_bodiess = robot.NumBodies;

    xlinks = zeros(1, num_bodiess);
    ylinks = zeros(1, num_bodiess);

    % robot links
    for i = 1:robot.NumBodies
        T = getTransform(robot, q, robot.Bodies{i}.Name);
        xlinks(i) = T(1,4);
        ylinks(i) = T(2,4);
    end
end

function [xee, yee] = ee_points(robot, q, ee_size)
    
    x_width = ee_size(1);
    y_width = ee_size(2);

    T = getTransform(robot, q, robot.Bodies{end}.Name);
%     base_point = T(:,4);

    p1 = T*([0; y_width/2; 0; 1]);
    p2 = T*([x_width; y_width/2; 0; 1]);
    p3 = T*([0; -y_width/2; 0; 1]);
    p4 = T*([x_width; -y_width/2; 0; 1]);

    xee = [p1(1), p2(1), p3(1), p4(1)];
    yee = [p1(2), p2(2), p3(2), p4(2)];
end


function [q_des, qd_des, qdd_des] = desired_trajectory(P, q_0, q_dot_0, q_ddot_0, t, k)
    % at a given time t and traj. param k value, return
    % the desired position, velocity, and acceleration.
    switch P.traj_type
    case 'orig'
        t_plan = P.t_plan;
        t_stop = P.t_stop;
        k_scaled = P.jrs_info.c_k_orig + P.jrs_info.g_k_orig.*k;
        
        if ~isnan(k)
            if t <= t_plan
                % compute first half of trajectory
                q_des = q_0 + q_dot_0*t + (1/2)*k_scaled*t^2;
                qd_des = q_dot_0 + k_scaled*t;
                qdd_des = k_scaled;
            else
                % compute trajectory at peak
                q_peak = q_0 + q_dot_0*t_plan + (1/2)*k_scaled*t_plan^2;
                qd_peak = q_dot_0 + k_scaled*t_plan;

                % compute second half of trajectory
                q_des = q_peak + qd_peak*(t-t_plan) + (1/2)*((0 - qd_peak)/(t_stop - t_plan))*(t-t_plan)^2;
                qd_des = qd_peak + ((0 - qd_peak)/(t_stop - t_plan))*(t-t_plan);
                qdd_des = (0 - qd_peak)/(t_stop - t_plan);
            end
        else
            % bring the trajectory to a stop in t_plan seconds
            % trajectory peaks at q_0
            q_peak = q_0;
            qd_peak = q_dot_0;
            
            if t <= t_plan && ~all(q_dot_0 == 0) % we're braking!
                q_des = q_peak + qd_peak*t + (1/2)*((0 - qd_peak)/t_plan)*t^2;
                qd_des = qd_peak + ((0 - qd_peak)/t_plan)*t;
                qdd_des = (0 - qd_peak)/t_plan;
            else % we should already be stopped, maintain that.
                q_des = q_peak;
                qd_des = zeros(size(q_dot_0));
                qdd_des = zeros(size(q_0));
            end
        end
    case 'bernstein'
        % assuming K = [-1, 1] corresponds to final position for now!!
        n_q = length(q_0);
        if ~isnan(k)
            q1 = q_0 + P.jrs_info.c_k_bernstein + P.jrs_info.g_k_bernstein.*k;
            for j = 1:n_q
                beta{j} = match_deg5_bernstein_coefficients({q_0(j); q_dot_0(j); q_ddot_0(j); q1(j); 0; 0});
                alpha{j} = bernstein_to_poly(beta{j}, 5);
            end
            q_des = zeros(length(q_0), 1);
            qd_des = zeros(length(q_0), 1);
            qdd_des = zeros(length(q_0), 1);
            for j = 1:n_q
                for coeff_idx = 0:5
                    q_des(j) = q_des(j) + alpha{j}{coeff_idx+1}*t^coeff_idx;
                    if coeff_idx > 0
                        qd_des(j) = qd_des(j) + coeff_idx*alpha{j}{coeff_idx+1}*t^(coeff_idx-1);
                    end
                    if coeff_idx > 1
                        qdd_des(j) = qdd_des(j) + (coeff_idx)*(coeff_idx-1)*alpha{j}{coeff_idx+1}*t^(coeff_idx-2);
                    end
                end
            end
        else
            % bring the trajectory to a stop using previous trajectory...
            t_plan = P.t_plan;
            if t <= t_plan && norm(q_dot_0) > 1e-8 && norm(q_dot_0) > 1e-8
                % just plug into previous trajectory, but shift time forward by t_plan.
                [q_des, qd_des, qdd_des] = P.info.desired_trajectory{end - 1}(t + t_plan);
            else % we should already be stopped, maintain that.
                q_des = q_0;
                qd_des = zeros(n_q, 1);
                qdd_des = zeros(n_q, 1);
            end
        end
    otherwise
        error('trajectory type not recognized');
    end
end

function [xp,yp] = circle(d,x,y,ang_start,ang_end,step)
    ang=ang_start:step:ang_end; 
    xp=(d/2)*cos(ang);
    yp=(d/2)*sin(ang);
end

function make_animation( h,index,filename )
    drawnow
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if index == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.001);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.001);
    end
end

function plotConcentricCircles(fig, inner_radius, outer_radius, fill_color, edge_color)
% PLOTCONCENTRICCIRCLES plots two concentric circles with specified properties.
%   plotConcentricCircles(fig, inner_radius, outer_radius, fill_color, edge_color)
%
%   Inputs:
%       fig:          Figure handle where the circles will be plotted.
%       inner_radius: Inner radius of the donut (hole).
%       outer_radius: Outer radius of the donut (edge).
%       fill_color:   Color to fill the donut (e.g., 'r' for red).
%       edge_color:   Color of the donut's edge (e.g., 'b' for blue).

    % Switch to the specified figure or create a new one if not provided
    if nargin < 1
        fig = figure;
    else
        figure(fig); 
    end
    
    hold on;
    
    rectangle('Position',[-outer_radius,-outer_radius,2*outer_radius,2*outer_radius],...
        'Curvature',[1,1],...
        'FaceColor',fill_color,...
        'EdgeColor',edge_color); % Set the face color of the outer circle

    % Plot the inner circle
    rectangle('Position',[-inner_radius,-inner_radius,2*inner_radius,2*inner_radius],...
        'Curvature',[1,1],...
        'FaceColor','w',...     % Set the face color of the inner circle to white (hole)
        'EdgeColor',edge_color); % Set the edge color of the inner circle

%     hold off;

%     % Set axis limits to better view the donut
%     axis equal;
%     xlim([-outer_radius-1, outer_radius+1]);
%     ylim([-outer_radius-1, outer_radius+1]);
% 
%     % Set labels and title
%     xlabel('X');
%     ylabel('Y');
%     title('Concentric Circles');
% 
%     % Display the grid for better visualization (optional)
%     grid on;
end