close all

%%% for agent
agent_urdf = 'Kinova_Grasp_w_Tray_Gripper.urdf';

add_uncertainty_to = 'all'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

agent_move_mode = 'integrator' ; % pick 'direct' or 'integrator'
use_CAD_flag = true; % plot robot with CAD or bounding boxes

%% robot params:
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
% params = load_robot_params(robot, ...
%                            'add_uncertainty_to', add_uncertainty_to, ...
%                            'links_with_uncertainty', links_with_uncertainty,...
%                            'uncertain_mass_range', uncertain_mass_range);
% joint_speed_limits = [-1.3963, -1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218;
%                        1.3963,  1.3963,  1.3963,  1.3963,  1.2218,  1.2218,  1.2218]; % matlab doesn't import these from urdf so hard code into class
% joint_input_limits = [-56.7, -56.7, -56.7, -56.7, -29.4, -29.4, -29.4;
%                        56.7,  56.7,  56.7,  56.7,  29.4,  29.4,  29.4]; % matlab doesn't import these from urdf so hard code into class
% transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class
% M_min_eigenvalue = 8.29938; % matlab doesn't import these from urdf so hard code into class

copyrobot = copy(robot);

%% Plotting

num_poses = 5;
travel = deg2rad(12);
start_pose = [-pi/2+travel/2;-pi/6;0;-3*pi/4;0;5*pi/12;0];
goal_pose = [-pi/2-travel/2;-pi/2;0;0;0;0;0];
% poses = linspace(start_pose,goal_pose,num_poses);

% % clf(101)
% figure(101)
% hold on
% view(3)
% % plotting poses
% % for i = 1:num_poses
% show(robot,[-pi/4+travel/2;-pi/2;0;0;0;0;0])
% show(robot,[-pi/4-travel/2;-pi/2;0;0;0;0;0])
% % end
% % plot formatting
% 
% grid off

figure(202)
% plot robot in start pose
ax = show(robot,start_pose,'Frames','off',PreservePlot=false,FastUpdate=true);
% find patch to adjust transparency
mesh_name = 'object_link';
rbtpatches=findobj(ax.Children,'Type','patch','-regexp','DisplayName',mesh_name);
set(rbtpatches,'FaceAlpha',0.6);
% set(rbtpatches,'FaceColor',[1 0 0]);
view(-37.5,20)
xlim([-0.5 0.5])
ylim([-0.9 0.15])
zlim([0 1])
grid off
material dull
% lighting gouraud
% camlight
lightangle(-37.5,20)
% zoom(1.5)