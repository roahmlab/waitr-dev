initialize_script_path = matlab.desktop.editor.getActiveFilename;
cd(initialize_script_path(1:end-12));

%% Initialize controller
fprintf("Start compiling mex robust controller\n\n");

% default robot to test
kinova_model_filename = 'kinova_with_gripper';
fprintf("You are using this urdf: %s \n",kinova_model_filename);

robot_model_file_path_filename = [pwd, '/kinova_simulator_interfaces/kinova_robust_controllers_mex/robot_model_file_path.hpp'];
fid = fopen(robot_model_file_path_filename, 'w');
fprintf(fid, 'const char RobotFilePath[] = "%s/kinova_simulator_interfaces/kinova_robust_controllers_mex/%s.txt";', pwd, kinova_model_filename);
fclose(fid);

cd kinova_simulator_interfaces/kinova_robust_controllers_mex/

try
    compile;
catch
    error('Error when compiling mex robust controller code!');
end

fprintf("Successfully compiled mex robust controller\n\n");

cd ../../

%% Initialize real-time force-constraint WAITR planner
fprintf("\nStart compiling WAITR\n\n");

cd kinova_simulator_interfaces/kinova_planner_realtime

armour_buffer_path = [pwd, '/BufferPath.h'];
fid = fopen(armour_buffer_path, 'w');
fprintf(fid, '#include <string>\n');
fprintf(fid, 'const std::string pathname = "%s/buffer/";', pwd);
fclose(fid);

terminal_output = system('env -i bash -i -c "./compile.sh"');

if terminal_output ~= 0
    error('Error when compiling WAITR real time planner code!');
else
    fprintf("Successfully compiled WAITR\n\n");
end

cd ../../

%% Initialize real-time WAITR planner
fprintf("\nStart compiling WAITR\n\n");

cd kinova_simulator_interfaces/kinova_planner_realtime_original

armour_buffer_path = [pwd, '/BufferPath.h'];
fid = fopen(armour_buffer_path, 'w');
fprintf(fid, '#include <string>\n');
fprintf(fid, 'const std::string pathname = "%s/buffer/";', pwd);
fclose(fid);

terminal_output = system('env -i bash -i -c "./compile.sh"');

if terminal_output ~= 0
    error('Error when compiling WAITR real time planner code!');
else
    fprintf("Successfully compiled WAITR\n\n");
end

cd ../../

%% Initialize sample based high level planner
fprintf("Start compiling sample based HLP\n\n");

cd kinova_simulator_interfaces/kinova_samplebased_HLP_realtime

HLP_buffer_path = [pwd, '/BufferPath.h'];
fid = fopen(HLP_buffer_path, 'w');
fprintf(fid, '#include <string>\n');
fprintf(fid, 'const std::string pathname = "%s/";', pwd);
fclose(fid);

terminal_output = system('env -i bash -i -c "./compile_collision_checker.sh"');

if terminal_output ~= 0
    error('Error when compiling ARMTD real time planner code!');
else
    fprintf("Successfully compiled HLP\n\n");
end

cd ../../

%% Save current folder path as .mat file for other scripts to read
kinova_test_folder_path = pwd;

save('kinova_test_folder_path.mat', 'kinova_test_folder_path');

%% Create dir
mkdir kinova_simulator_interfaces/kinova_planner_realtime/buffer/
addpath kinova_simulator_interfaces/kinova_planner_realtime/buffer/
mkdir kinova_simulator_interfaces/kinova_planner_realtime_armtd_comparison/buffer/
addpath kinova_simulator_interfaces/kinova_planner_realtime_armtd_comparison/buffer/
