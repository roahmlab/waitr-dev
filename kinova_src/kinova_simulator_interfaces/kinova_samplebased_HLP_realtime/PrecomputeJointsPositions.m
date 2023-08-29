clear; clc;

sample_nodes = load('QConfig_composite_hardware_only.txt');
sample_nodes(:,1) = wrapToPi(sample_nodes(:,1)+pi/2);
q_valid_list = sample_nodes';

robot = importrobot('Kinova_Grasp_URDF.urdf');
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot);

%% precompute joint positions based on sampled configurations
NUM_NODES = size(q_valid_list,2);

joint_positions = zeros(NUM_NODES * 9, 3);

for i = 1:NUM_NODES
    q = q_valid_list(:,i);
    joint_position_series = forward_kinematics_modified([q; 0; 0; 0], params.nominal.T0, params.nominal.joint_axes);
    joint_positions( (i-1)*9+1 : i*9, : ) = joint_position_series(:,[1:8,10])';
end

%% write to csv file
writematrix(joint_positions, 'joint_positions_composite_hardware_only.csv', 'Delimiter', ' ');