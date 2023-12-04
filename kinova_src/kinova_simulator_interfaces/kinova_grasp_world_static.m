classdef kinova_grasp_world_static < world
    properties
        % setup info
        include_base_obstacle = false;
        N_random_obstacles = 0 ;
        create_random_obstacles_flag = true ;
        obstacle_size_range = [0.01 0.5] ; % [min, max] side length
        create_configuration_timeout = 200 ;
        create_obstacle_timeout =  1200 ;
        min_dist_in_config_space_between_start_and_goal
        workspace_goal_check = 0;

        % grasp flags
        grasp_constraint_flag = false;
        ik_start_goal_flag = false;
        % contact info 
        u_s = NaN;
        surf_rad = NaN;

        robot;

        % arm info
        arm_joint_state_limits
        arm_n_links_and_joints
        arm_joint_state_indices
        
        % goal
        goal_type = 'configuration' ;
        goal_buffer_distance = 0.2 ; % m
        goal_in_workspace
        
        % goal plotting
        goal_plot_patch_data
        start_plot_patch_data
        goal_plot_face_color = [0 1 0] ;
        goal_plot_face_alpha = 0.1 ;
        goal_plot_edge_color = [0 1 0] ;
        goal_plot_edge_alpha = 0.3 ;
        goal_plot_edge_style = '--' ;
        
        % obstacle creation buffer distance
        creation_buffer = 0 ; % distance in m from closest obstacle or link
        base_creation_buffer = 0 ;
        setting_up = false ; % change this flag when setting up start/goal config, obstacles

        robot_params
    end
    
    methods
        %% constructor
        function W = kinova_grasp_world_static(varargin)
            % W = arm_world_static('Aproperty1',value1,'property2',value2,...)
            
            default_goal_radius = 0.05 ; % rad/joint
            W@world('start',[],'goal',[],...
                'goal_radius',default_goal_radius,...
                varargin{:}) ;%'N_obstacles',0,...
            
            W.plot_data.obstacles = [] ;
            W.plot_data.goal = [] ;
%             W.setup() ;
        end
        
        %% setup
        function setup(W,I)
            W.vdisp('Running arm world setup',1)
            
            % generate obstacles representing the fetch's body
            if W.include_base_obstacle
                W.create_base_obstacle() ;
            end
            
            if nargin > 1
                % make sure the world's properties agree with the arm's
                W.get_arm_info(I)
                
                W.setting_up = true;
                
                % create a start configuration
                if isempty(W.start)
                    W.create_start(I) ;
                end
                
                % create a goal configuration
                if isempty(W.goal)
                    W.create_goal(I) ;
                end
                
                if strcmp(W.goal_type,'configuration')
                    W.goal_plot_patch_data = I.get_collision_check_volume(W.goal) ;
                    W.start_plot_patch_data = I.get_collision_check_volume(W.start) ;
                end
                
                % create random obstacles
                if W.create_random_obstacles_flag && (W.N_random_obstacles > 0)
                    for idx = 1:W.N_random_obstacles
                        O = W.create_collision_free_obstacle(I) ;
                        if ~isempty(O)
                            W.add_obstacle(O) ;
                        end
                    end
                end
            else
                W.vdisp('Please run W.setup with agent info!',5)
            end
            
            % update the number of obstacles
            W.N_obstacles = length(W.obstacles) ;
            
            % make goal in workspace as the joint locations
            switch W.goal_type
                case 'configuration'
                    W.vdisp('Making goal in workspace using joint locations',5)
                    W.goal_in_workspace = W.goal ;
                case 'end_effector_location'
                    W.vdisp('Setting end effector position as goal in workspace!',5)
                    J = I.get_joint_locations(W.goal) ;
                    W.goal_in_workspace = J(:,end) ;
                case 'fk_func'
                    % the difference between this case and the
                    % 'end_effector_location' case is that this uses the
                    % forward_kinematics.m function which currently only
                    % does forward kinematics for actuated joints.
                    W.vdisp('Setting goal as forward_kinematics(q) result',5)
                    fk_result = forward_kinematics(W.goal,W.robot_params.true.T0,W.robot_params.true.joint_axes);
                    W.goal_in_workspace = fk_result(1:3,4);
            end
            W.vdisp('Arm world setup complete',2)
                    
            
            W.setting_up = false;
        end
        
        function add_obstacle(W,O)
            if iscell(O)
                W.obstacles = [W.obstacles, O] ;
            else
                W.obstacles = [W.obstacles, {O}] ;
            end
            W.N_obstacles = length(W.obstacles) ;
        end
        
        %% get bounds and joint limits
        function get_arm_info(W,I)
            % update the world's dimension based on the arm's dimension
            if W.dimension ~= I.dimension
                W.vdisp(['Updating world dimension to ',num2str(I.dimension),...
                    ' to match the arm!'],3)
                W.dimension = I.dimension ;
            end
            
            % set world bounds based on agent limits
            W.bounds = I.reach_limits ;
            
            % set any joint limits that are +Inf to 2*pi and -Inf to -2*pi
            % changed to +- 1000 to match cuda code
            % do -pi/pi for rtd-force so last joint doesn't have to go all the way around
            joint_state_limits = I.joint_state_limits ;
            joint_limit_infs = isinf(joint_state_limits) ;
            joint_state_limits(1,joint_limit_infs(1,:)) = -pi/2; % restricting to prevent large rotations hopefully
            joint_state_limits(2,joint_limit_infs(2,:)) = pi/2;
            
            W.arm_joint_state_limits = joint_state_limits ;
            W.arm_n_links_and_joints = size(joint_state_limits,2) ;
            W.arm_joint_state_indices = I.joint_state_indices ;
            
            % set minimum distance between start and goal based on the
            % joint limits
            joint_ranges = diff(joint_state_limits,[],1) ;
            W.min_dist_in_config_space_between_start_and_goal = norm(0.10*joint_ranges) ;
        end
        
        %% make start and goal
        function create_start(W,I)
            W.vdisp('Making start configuration',5)
            W.start = W.create_collision_free_configuration(I) ;
            if isempty(W.start)
                W.vdisp('Using agent current pose as start config',3)
                W.start = I.state(I.joint_state_indices,end) ;
            end
        end
        
        function create_goal(W,I)
            W.vdisp('Making goal!',3)
            
            switch W.goal_type
                case 'configuration'
                    W.vdisp('Making goal configuration',5)
                    
                    dist_between_start_and_goal = 0 ;
                    start_tic = tic ;
                    t_cur = toc(start_tic) ;
                    new_goal = [] ;
                    
                    while dist_between_start_and_goal < W.min_dist_in_config_space_between_start_and_goal && ...
                            t_cur <= W.create_configuration_timeout
                        
                        % new_goal = rand_range(W.arm_joint_state_limits(1,:),W.arm_joint_state_limits(2,:))' ;
                        new_goal = W.create_collision_free_configuration(I) ;
                        
                        dist_between_start_and_goal = norm(W.start - new_goal) ;
                        
                        t_cur = toc(start_tic) ;
                    end
                    
                    if isempty(new_goal)
                        W.vdisp('Goal creation failed! Using random goal',3)
                        W.goal = rand_range(W.arm_joint_state_limits(1,:),W.arm_joint_state_limits(2,:))' ;
                    else
                        W.goal = new_goal ;
                    end
                case 'end_effector_location'
                    % make a position that isn't in any obstacles
                    goal_in_obs = true ;
                    while goal_in_obs
                        B = W.bounds ;
                        B = reshape(B(:),2,[])' ;
                        new_goal = rand_range(B(:,1),B(:,2)) ;
                        
                        dists_to_obs = zeros(1,length(W.obstacles)) ;
                        
                        for idx = 1:length(W.obstacles)
                            o = W.obstacles{idx} ;
                            if isa(o,'box_obstacle_zonotope')
                                
                                dists_to_obs(idx) = dist_point_to_box(new_goal,...
                                    o.side_lengths(1),...
                                    o.side_lengths(2),...
                                    o.side_lengths(3),...
                                    o.center) ;
                                
                            end
                        end
                        
                        d = min(dists_to_obs) ;
                        if d < W.goal_buffer_distance
                            goal_in_obs = false ;
                        end
                    end
                    
                    if ~isempty(new_goal)
                        W.goal = new_goal ;
                    else
                        error('goal creation failed!')
                    end
                case 'fk_func'
                    % make a position that isn't in any obstacles
                    goal_in_obs = true ;
                    while goal_in_obs
                        B = W.bounds ;
                        B = reshape(B(:),2,[])' ;
                        new_goal = rand_range(B(:,1),B(:,2)) ;
                        
                        dists_to_obs = zeros(1,length(W.obstacles)) ;
                        
                        for idx = 1:length(W.obstacles)
                            o = W.obstacles{idx} ;
                            if isa(o,'box_obstacle_zonotope')
                                
                                dists_to_obs(idx) = dist_point_to_box(new_goal,...
                                    o.side_lengths(1),...
                                    o.side_lengths(2),...
                                    o.side_lengths(3),...
                                    o.center) ;
                                
                            end
                        end
                        
                        d = min(dists_to_obs) ;
                        if d < W.goal_buffer_distance
                            goal_in_obs = false ;
                        end
                    end
                    
                    if ~isempty(new_goal)
                        W.goal = new_goal ;
                    else
                        error('goal creation failed!')
                    end
                otherwise
                    error('Invalid goal_type property!')
            end
        end
        
        %% make configurations
        function q = create_collision_free_configuration(W,I)
            config_is_valid = false ;
            start_tic = tic ;
            t_cur = toc(start_tic) ;

            obstacle_check = false;
            grasp_check = false;
            
            while ~config_is_valid && t_cur <= W.create_configuration_timeout
                q = W.create_random_configuration() ;
%                 config_is_valid = ~(W.collision_check_single_state(I,q)) ;
                obstacle_check = ~(W.collision_check_single_state(I,q)); % returns true if collision, so false if no collision

                if W.grasp_constraint_flag
                    % check that grasp constraints are satisfied
                    grasp_check = ~(W.grasp_check_single_state_resting(I,q));
                    % also check that joint limits are satisfied
%                     grasp_check2 = ~(W.joint_limit_check(I,q));
%                     if ~grasp_check1 || ~grasp_check2
%                         grasp_check = false;
%                     end
                else
                    grasp_check = false; % other function returns true if violation, which is then flipped in the above statement to be a false. so want true here.
                end

                if (~obstacle_check) || (~grasp_check)
                    config_is_valid = false; % switch?
                else
                    config_is_valid = true;
                end

                t_cur = toc(start_tic) ;
            end
            
            if ~config_is_valid
                q = [] ;
                W.vdisp('Configuration creation failed!',3)
            end
        end
        
        function q = create_random_configuration(W)
            % use IK here to generate random config with tray upright
            if W.ik_start_goal_flag
                % setting up IK solver
                ik = robotics.InverseKinematics('RigidBodyTree',W.robot);
                weights = [0.25 0.25 0.25 1 1 1];
                initialguess = homeConfiguration(W.robot);
                % choose random x,y,z components for location of 10th joint
                % in taskspace
                rand_x = rand_range(-0.85,0.85);
                rand_y = rand_range(-0.85,0.85);
                rand_z = rand_range(-0.2,0.85);

                %% ! Note: need to also vary the x,y rotation? or the rotation
                % matrix altogether, and then just set the z-rotation to
                % vertical. could just vary the first 2x2 of the rotation
                % matrix to get random rotation.
                out = true;
                while out == true
                % solving for a config other than home config
                    [q, solInfo1] = ik('cube_link',[1 0 0 rand_x; 0 1 0 rand_y; 0 0 1 rand_z; 0 0 0 1],weights,initialguess);
                    % put check for config in joint limits here?
                    
%                     q = mod(q,pi);

                    q_lower = W.arm_joint_state_limits(1,:)+.1; % + 0.01; % tighten the bounds to make sure valid
                    q_upper = W.arm_joint_state_limits(2,:)-.1; % + 0.01; % tighten the bounds to make sure valid


                    for i = 1:length(q)
%                         test = append(num2str(q_lower(i)),' ',num2str(q(i)),' ',num2str(q_upper(i)));
%                         disp(test)
                        if (q(i) < q_lower(i)) || (q(i) > q_upper(i))
                            out = true;
                            break
                        else
                            out = false;
                        end
                    end
                end

            else
                q = rand_range(W.arm_joint_state_limits(1,:)+0.025,W.arm_joint_state_limits(2,:)-0.025)' ;
            end
        end
        
        %% make obstacles
        function O = create_collision_free_obstacle(W,I,q)
            if nargin < 3
                q = W.start ;
                q2 = W.goal ;
            end
            
            obstacle_is_valid = false ;
            start_tic = tic ;
            t_cur = toc(start_tic) ;
            V_arm = I.get_collision_check_volume(q) ;
            
            if strcmp(W.goal_type,'configuration')
                V_arm2 = I.get_collision_check_volume(q2) ;
            end
            
            while ~obstacle_is_valid && t_cur <= W.create_obstacle_timeout
                O = W.create_random_obstacle() ;
                obstacle_is_valid = ~(W.collision_check_single_obstacle(O,V_arm)) && ...
                    ~(W.collision_check_current_obstacles(O));
                
                if strcmp(W.goal_type,'configuration')
                    obstacle_is_valid = obstacle_is_valid && ...
                        ~(W.collision_check_single_obstacle(O,V_arm2)) ;
                end
                
                t_cur = toc(start_tic) ;
            end
            
            if ~obstacle_is_valid
                O = [] ;
                W.vdisp('Obstacle creation failed! Try again...',3)
            end
        end
        
        function O = create_random_obstacle(W)
            % create center
            B = W.bounds ;
%             B(1) = 0; B(2) = 1; B(3) = -0.5; B(4) = 0.5; B(5) = 0; B(6) = 1;
            center = [rand_range(B(1)+W.obstacle_size_range(2)/2,B(2)-W.obstacle_size_range(2)/2) ; rand_range(B(3)+W.obstacle_size_range(2)/2,B(4)-W.obstacle_size_range(2)/2)] ;
%             center = [rand_range(0,B(2)) ; rand_range(B(3),B(4))] ;
            
            if W.dimension == 3
                center = [center ; rand_range(B(5)+W.obstacle_size_range(2)/2,B(6)-W.obstacle_size_range(2)/2)] ;
            end
            
            % create side lengths
            side_lengths = rand_range(W.obstacle_size_range(1),...
                W.obstacle_size_range(2),[],[],1,W.dimension) ;
            
            % create obstacle
            O = box_obstacle_zonotope('center',center(:),...
                'side_lengths',side_lengths, ...
                'creation_buffer', W.creation_buffer) ;
        end
        
        function create_base_obstacle(W)
            W.vdisp('Making base obstacles',3) ;
            
            % floor zono
            floor_center = [-0.0331;0;0.005];
            floor_sides = 2*[1.3598, 1.3598, 0.0025];
            
            % base zono
            base_center = [-0.0580; 0; 0.1778];
            base_sides = 2*[0.2794, 0.2794, 0.1778];
            
            % tower zono
            tower_center = [-0.2359; 0; 0.6868];
            tower_sides = 2*[0.1016, 0.1651, 0.3312];
            
            % head zono
            head_center = [-0.0580; 0; 1.0816];
            head_sides = 2*[0.1651, 0.1397, 0.0635];
            
            floor_face_color = [0.9 0.9 0.9];
            plot_face_color = [0.5 0.5 0.5];
            plot_edge_color = [0 0 0];
            
            O_floor =  box_obstacle_zonotope('center',floor_center,...
                'side_lengths',floor_sides,...
                'plot_face_color',floor_face_color,...
                'plot_edge_color',plot_edge_color,...
                'is_base_obstacle',true, ...
                'creation_buffer',W.base_creation_buffer) ;
            
            W.obstacles = [W.obstacles, {O_floor}];
            W.N_obstacles = length(W.obstacles) ;
        end
        
        %% collision checking
        function out = collision_check_single_state(W,I,q)
            % out = collision_check_single_state(W,agent_info,agent_state)
            %
            % Run a collision check for the given state and return true if
            % it is in collision. This gets called by W.collision_check.
            
%             O = W.obstacles(1:end-1) ; % temporarily ignore ground obstacles
%             N_O = length(O) ; % in case W.N_obstacles is wrong
%             out = false ; % optimism!
%             o_idx = 1 ;
%             V_arm = I.get_collision_check_volume(q) ;
%             
%             while (o_idx <= N_O) && ~out
%                 O_idx = O{o_idx} ;
%                 out = W.collision_check_single_obstacle(O_idx,V_arm) ;
% 
% %                 if out
% %                     disp(q);
% %                     disp(O_idx.Z);
% %                 end
% 
%                 o_idx = o_idx + 1 ;
%             end
            out = false;
        end
        
        function out = collision_check_single_obstacle(W,obstacle_object,arm_volume)
            switch W.dimension
                case 2
                    obstacle_object = obstacle_object.collision_check_patch_data.vertices ;
                    obstacle_object = [obstacle_object ; obstacle_object(1,:)] ;
                    [x_int,~] = polyxpoly(arm_volume(1,:)',arm_volume(2,:)',...
                        obstacle_object(:,1),obstacle_object(:,2)) ;
                    if isempty(x_int)
                        out = false ;
                    else
                        out = true ;
                    end
                case 3
                    if W.setting_up
                        O_str.faces = obstacle_object.buffered_collision_check_patch_data.faces ;
                        O_str.vertices = obstacle_object.buffered_collision_check_patch_data.vertices ;
                        check = SurfaceIntersection(O_str,arm_volume) ;
                        out = any(check(:)) || inhull(obstacle_object.center', arm_volume.vertices) ;
                    else
                        O_str.faces = obstacle_object.collision_check_patch_data.faces ;
                        O_str.vertices = obstacle_object.collision_check_patch_data.vertices ;
                        check = SurfaceIntersection(O_str,arm_volume) ;
                        out = any(check(:)) ;
                    end
            end
        end
        
        function out = collision_check_current_obstacles(W, obstacle_object)
            switch W.dimension
                case 2
                    error('not implemented yet!');
%                     obstacle_object = obstacle_object.collision_check_patch_data.vertices ;
%                     obstacle_object = [obstacle_object ; obstacle_object(1,:)] ;
%                     [x_int,~] = polyxpoly(arm_volume(1,:)',arm_volume(2,:)',...
%                         obstacle_object(:,1),obstacle_object(:,2)) ;
%                     if isempty(x_int)
%                         out = false ;
%                     else
%                         out = true ;
%                     end
                case 3
                    O_str.faces = obstacle_object.buffered_collision_check_patch_data.faces ;
                    O_str.vertices = obstacle_object.buffered_collision_check_patch_data.vertices ;
                    out = false;
                    for i = 1:length(W.obstacles)
                       Ocurr_str.faces = W.obstacles{i}.collision_check_patch_data.faces ;
                       Ocurr_str.vertices = W.obstacles{i}.collision_check_patch_data.vertices ;
                       Ocurr_center = W.obstacles{i}.center;
                       check = SurfaceIntersection(O_str, Ocurr_str) ;
                       out_curr = any(check(:)) || inhull(obstacle_object.center', Ocurr_str.vertices) || inhull(Ocurr_center', O_str.vertices);
                       out = out || out_curr;
                    end
            end
        end

        %% Grasp Checking
        function out = grasp_check_single_state_resting(W,I,q)
            % out = collision_check_single_state(W,agent_info,agent_state)
            %
            % Run a grasp constraint check for the given state and return 
            % true if there is a violation.
            
            out = false;            

            %% Calculating forces
            % call RNEA on current configuration (assuming at rest? cannot)
            % (also assuming use_gravity is true)
            [tau, f, n] = rnea(q,zeros(length(q),1),zeros(length(q),1),zeros(length(q),1),true,I.params.true);
            
            fx = f(1,10);
            fy = f(2,10);
            fz = f(3,10);
            
            %% Calculating Constraints (written as <0)
            
            % ZMP_Moment = n(:,10) + cross([0;0;cup_height],f(:,10));
            
            sep = -1*fz; %fz; %
            slip = sqrt(fx^2+fy^2) - W.u_s*abs(fz) + 3.25; % offset to make initial/goal closer to horizontal
            ZMP = cross([0;0;1],n(:,10))./dot([0;0;1],f(:,10));
%             ZMP = cross(ZMP_Moment,[0;0;1])./dot([0;0;1],f(:,10)); % RNEA
%             passes out the force and moment at the joint so original ZMP
%             was correct
            tip = sqrt(ZMP(1)^2 + ZMP(2)^2) - W.surf_rad; % offset to make initial/goal closer to horizontal
            
            if (sep > 0) || (slip > 0) || (tip > 0) % greater than zero is violation
                out = true;
            else
                test = [];
            end            
        end
        
        function [out, sep_val, slip_val, tip_val] = grasp_check_single_state(W,A,agent_info,planner_info,q,t_check,t_start,t_idx)
            % out = collision_check_single_state(W,agent_info,agent_state)
            %
            % Run a grasp constraint check for the given state and return 
            % true if there is a violation.

            if W.grasp_constraint_flag

                out = false;
                sep_val = false;
                slip_val = false;
                tip_val = false;
    
                [M, C, g] = A.calculate_dynamics(q(A.joint_state_indices), q(A.joint_speed_indices), agent_info.params.true);
                
                for i = 1:A.n_inputs
                    M(i,i) = M(i,i) + A.transmision_inertia(i);
                end

                % put this here?
                u = A.LLC.get_control_inputs(A, t_check-t_start, q, planner_info);
    
                % calculate acceleration 
                qdd = M\(u-C*q(A.joint_speed_indices)-g);
    
                %% Calculating forces
                % call RNEA on current configuration (assuming at rest? cannot)
                % (also assuming use_gravity is true)
                [tau, f, n] = rnea(q(A.joint_state_indices),q(A.joint_speed_indices),q(A.joint_speed_indices),qdd,true,agent_info.params.true);
                
                fx = f(1,10);
                fy = f(2,10);
                fz = f(3,10);
    
                %% Calculating Constraints (written as <0)
                sep = -1*fz; %fz; %
                slip = sqrt(fx^2+fy^2) - W.u_s*abs(fz);
                slip2 = fx^2+fy^2 - W.u_s^2*fz^2;
                ZMP = cross([0;0;1],n(:,10))./dot([0;0;1],f(:,10));
                tip = sqrt(ZMP(1)^2 + ZMP(2)^2) - W.surf_rad; % + tip_threshold;
                ZMP_top = cross([0;0;1],n(:,10));
                ZMP_bottom = dot([0;0;1],f(:,10));
                tip2 = ZMP_top(1)^2 + ZMP_top(2)^2 - W.surf_rad^2*ZMP_bottom^2;

                if (sep > 1e-6) || (slip2 > 1e-6) || (tip2 > 1e-6)
                    out = true;
                end
                
                if sep > 1e-6
                    sep_val = true;
                end
                
                if slip2 > 1e-6
                    slip_val = true;
                end
                
                if tip2 > 1e-6
                    tip_val = true;
                end
            else
                out = false;
                sep_val = false;
                slip_val = false;
                tip_val = false; 
            end
            
        end

        function [sep_val, slip_val, tip_val] = grasp_check(W,A,agent_info,planner_info)
            
            t_start = W.current_time;
            
            % create time vector for checking
            t_agent = agent_info.time(end);
            t_check = t_start:A.traj_check_time_discretization:t_agent; % just copied this so using same time discretization
            
            if isempty(t_check) || t_check(end) ~= t_agent
                t_check = [t_check, t_agent];
            end
            
            % get agent trajectory interpolated to time
            z_agent = match_trajectories(t_check,agent_info.time,agent_info.state);
            
            % run collision check
            W.vdisp('Running grasp checks!',3)
            out = false;
            sep_val = false; % optimism!
            slip_val = false; % more optimism!!
            tip_val = false; % even more optimism!!!
            t_idx = 1;
            
            while ~sep_val && ~slip_val && ~tip_val && t_idx < length(t_check) % 
                
                z = z_agent(:,t_idx);
                [out, sep_val, slip_val, tip_val] = W.grasp_check_single_state(A,agent_info,planner_info,z,t_check(t_idx),t_start,t_idx);  % agent_info,z);

                t_idx = t_idx + 1;
                
            end
            
            if out
                if sep_val
                    W.vdisp(['Grasp separation violation detected at t = ',num2str(t_check(t_idx))],1)
                end
                if slip_val
                    W.vdisp(['Grasp slipping violation detected at t = ',num2str(t_check(t_idx))],1)
                end
                if tip_val
                    W.vdisp(['Grasp tipping violation detected at t = ',num2str(t_check(t_idx))],1)
                end
                % for debugging, call check again
                [out, sep_val, slip_val, tip_val] = W.grasp_check_single_state(A,agent_info,planner_info,z,t_check(t_idx),t_start,t_idx);
                
            else
                W.vdisp('No grasp violations detected',3)
            end
        end

        function out = joint_limit_check(W,I,q)
            out = false;
            q_lower = W.arm_joint_state_limits(1,:);
            q_upper = W.arm_joint_state_limits(2,:);
            for i = 1:length(q)
                if (q(i) < q_lower(i)) || (q(i) > q_upper(i))
                    out = true;
                end
            end
        end
        
        
        %% goal check
        function out = goal_check(W,agent_info)
            
            z = agent_info.state(W.arm_joint_state_indices,end) ;
            
            switch W.goal_type
                case 'configuration'
                    dz = abs(angdiff(z - W.goal)) ;
                    dz_log = dz <= W.goal_radius ;
                    out = all(dz_log,1) ;
                case 'end_effector_location'
                    % get the joint locations
                    J = agent_info.get_joint_locations(z) ;
                    
                    if ~iscell(J)
                        J = {J};
                    end
                    
                    % concatenate all the end effector locations into one
                    % array
                    N_J = length(J) ;
                    J_ee = nan(3,N_J) ;
                    for idx = 1:N_J
                        J_ee(:,idx) = J{idx}(:,end) ;
                    end
                    
                    % check how far the end effector is from the goal
                    % location
                    dz = vecnorm(J_ee - repmat(W.goal_in_workspace,1,N_J)) ;
                    out = any(dz <= W.goal_radius) ;
                case 'fk_func'
                    % get the joint locations
                    ee_cur = forward_kinematics(agent_info.position(:,end),W.robot_params.true.T0,W.robot_params.true.joint_axes);
                    ee_cur_pos = ee_cur(1:3,4);
                    % check how far the end effector is from the goal
                    % location
                    dz = vecnorm(ee_cur_pos - W.goal_in_workspace) ;
                    out = any(dz <= W.goal_radius) ;

                otherwise
                    error(['The goal type ',W.goal_type,' is not supported!'])
            end
            %                 error('not implemented yet!');
            %%%PATRICK HACK FOR 1 LINK
            %                 z_agent = agent_info.state(W.arm_joint_state_indices,end) ;
            %                 z_goal = W.goal;
            %                 x_agent = make_orientation(z_agent(1), 3)*make_orientation(z_agent(2), 2)*[0.33; 0; 0];
            %                 x_goal = make_orientation(z_goal(1), 3)*make_orientation(z_goal(2), 2)*[0.33; 0; 0];
            %                 dz = min(sqrt(sum((x_agent - x_goal).^2)));
            %                 out = dz <= W.goal_radius;
            
                %%%PATRICK HACK FOR FETCH
%                 z_agent = agent_info.state(W.arm_joint_state_indices,end) ;
%                 z_goal = W.goal;
% %                 x_agent = make_orientation(z_agent(1), 3)*make_orientation(z_agent(2), 2)*[0.33; 0; 0];
%                 
%                 x_agent_1 = make_orientation(z_agent(1), 3)*make_orientation(z_agent(2), 2)*[0.33; 0; 0];
%                 x_agent_2 = x_agent_1 + make_orientation(z_agent(1), 3)*make_orientation(z_agent(2), 2)*make_orientation(z_agent(3), 1)*make_orientation(z_agent(4), 2)*[0.33; 0; 0];
%                 x_agent_3 = x_agent_2 + make_orientation(z_agent(1), 3)*make_orientation(z_agent(2), 2)*make_orientation(z_agent(3), 1)*make_orientation(z_agent(4), 2)*make_orientation(z_agent(5), 1)*make_orientation(z_agent(6), 2)*[0.33; 0; 0];
%                 x_agent = [x_agent_1; x_agent_2; x_agent_3];
%                 
% %                 x_goal = make_orientation(z_goal(1), 3)*make_orientation(z_goal(2), 2)*[0.33; 0; 0];
%                 x_goal_1 = make_orientation(z_goal(1), 3)*make_orientation(z_goal(2), 2)*[0.33; 0; 0];
%                 x_goal_2 = x_goal_1 + make_orientation(z_goal(1), 3)*make_orientation(z_goal(2), 2)*make_orientation(z_goal(3), 1)*make_orientation(z_goal(4), 2)*[0.33; 0; 0];
%                 x_goal_3 = x_goal_2 + make_orientation(z_goal(1), 3)*make_orientation(z_goal(2), 2)*make_orientation(z_goal(3), 1)*make_orientation(z_goal(4), 2)*make_orientation(z_goal(5), 1)*make_orientation(z_goal(6), 2)*[0.33; 0; 0];
%                 x_goal = [x_goal_1; x_goal_2; x_goal_3];
%                 
%                 dz = min(sqrt(sum((x_agent - x_goal).^2)));
%                 out = dz <= W.goal_radius;
%             end
            
        end
        
        %% plotting
        function plot(W)
            hold_check = ~ishold ;
            if hold_check
                hold on ;
            end
            
            % plot obstacles (each obstacle takes care of its own plotting)
            W.vdisp('Plotting obstacles',6)
            for idx = 1:W.N_obstacles
                plot(W.obstacles{idx}) ;
            end
            
            % plot goal config
            W.plot_goal()
            
            if hold_check
                hold off
            end
        end
        
        function plot_goal(W)
            W.vdisp('Plotting goal',6)
            
            hold_check = ~ishold ;
            if hold_check
                hold on ;
            end
            
            switch W.goal_type
                case 'configuration'
                    G = W.goal_plot_patch_data ;
                    if ~check_if_plot_is_available(W,'goal') && ~isempty(W.goal)
                        switch W.dimension
                            case 2
                                data = plot(G(1,:),G(2,:),'Color',W.goal_plot_edge_color,...
                                    'LineStyle',W.goal_plot_edge_style) ;
                            case 3
                                data = patch(G,...
                                    'LineStyle',W.goal_plot_edge_style,...
                                    'FaceColor',W.goal_plot_face_color,...
                                    'FaceAlpha',W.goal_plot_face_alpha,...
                                    'EdgeColor',W.goal_plot_edge_color,...
                                    'EdgeAlpha',W.goal_plot_edge_alpha) ;

%                                 data_another = patch(W.start_plot_patch_data,...
%                                     'LineStyle',W.goal_plot_edge_style,...
%                                     'FaceColor',W.goal_plot_face_color,...
%                                     'FaceAlpha',W.goal_plot_face_alpha,...
%                                     'EdgeColor',W.goal_plot_edge_color,...
%                                     'EdgeAlpha',W.goal_plot_edge_alpha) ;
                        end
                        
                        W.plot_data.goal = data ;
                    end
                case 'end_effector_location'
                    g = W.goal_in_workspace ;
                    if ~check_if_plot_is_available(W,'goal') && ~isempty(W.goal)
                        data = plot_path(g,'p','Color',W.goal_plot_edge_color,'LineWidth',2) ;
                        W.plot_data.goal = data ;
                    end
                case 'fk_func'
                    g = W.goal_in_workspace ;
                    if ~check_if_plot_is_available(W,'goal') && ~isempty(W.goal)
                        data = plot_path(g,'p','Color',W.goal_plot_edge_color,'LineWidth',2) ;
                        W.plot_data.goal = data ;
                    end
            end
        
            if hold_check
                hold off
            end
        end
    end
end