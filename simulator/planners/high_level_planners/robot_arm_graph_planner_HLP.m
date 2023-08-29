classdef robot_arm_graph_planner_HLP < high_level_planner
    %% Description
    % High level planner that takes in a list of waypoints that are nodes
    % in a graph. This planner returns a waypoint using a straight line
    % planner to get to the current closest passed in waypoint.
    
    %% properties
    properties

        % arm
        arm_n_states
        arm_n_inputs
        arm_n_links_and_joints
        arm_joint_state_limits
        arm_joint_speed_limits
        arm_joint_input_limits
        arm_joint_state_indices
        arm_joint_speed_indices
        
        % samples
%         nodes
%         adjacency_matrix
%         n_nodes
%         n_nodes_max = 20000 ;
%         all_node_idxs ;
%         sampling_timeout = 0.1 ; % seconds per planning iteration
        
        % path
%         best_path_nodes
%         best_path_node_idxs

        graph
        graph_waypoints
        current_graph_waypoint_index = 0;
        current_waypoint_patch_data = [];
        next_waypoint_patch_data = [];
        previous_waypoint_patch_data = [];
        q_des = [];
        waypoint_counter = 0;
        max_waypoint_counter = 15;

        nearly_reach_goal = false;
    end
    methods
        %%  constructor
        function HLP = robot_arm_graph_planner_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
        end
        
        %% setup
        function setup(HLP,agent_info,world_info)
            % get all the necessary arm properties filled in
            HLP.vdisp('Filling in HLP''s arm properties',9)
            HLP = fill_in_arm_properties(HLP,agent_info,false) ;
            
            % get the world goal
            HLP.vdisp('Setting goal',9)
            HLP.goal = world_info.goal ;
        end
        
        %% get waypoint
        function waypoint = get_waypoint(HLP,agent_info,world_info,use_SLP,SLP_lookahead_distance,increment_waypoint_distance)
            % tracking number of times one graph waypoint is tracked
            HLP.waypoint_counter = HLP.waypoint_counter + 1;

            q_cur = agent_info.state(HLP.arm_joint_state_indices, end);

            if isempty(HLP.current_waypoint) % meaning just starting

                % increment current graph waypoint index
%                 graph_waypoint = HLP.graph_waypoints(:,1);
                HLP.current_graph_waypoint_index = 1;

                % assign goal using graph waypoints
                new_goal = HLP.graph_waypoints(:,HLP.current_graph_waypoint_index);

                % call Straight Line Planner function to get waypoint in
                % direction of graph waypoint goal
                if use_SLP
                    waypoint = get_SLP_to_waypoint(HLP,new_goal,agent_info,SLP_lookahead_distance); % new_goal; % 
                else 
                    waypoint = new_goal;
                    % update current waypoints
                    HLP.current_waypoint = waypoint;
                    HLP.waypoints = [HLP.waypoints, waypoint];
                    HLP.current_waypoint_index = HLP.current_waypoint_index + 1;
                end
            else

                if HLP.waypoint_counter < HLP.max_waypoint_counter
                    % check how close to current waypoint
                    cur_dist = norm(angdiff(q_cur, HLP.graph_waypoints(:,HLP.current_graph_waypoint_index)));
    
                    fprintf('        HLP: Current distance to waypoint: %f\n', cur_dist);
    
                    % decide waypoint based on distance
                    if ~HLP.nearly_reach_goal && cur_dist > increment_waypoint_distance
                        % if too far away, return SLP waypoint based on current
                        % graph waypoint
                        new_goal = HLP.graph_waypoints(:,HLP.current_graph_waypoint_index);
                        if use_SLP
                            waypoint = get_SLP_to_waypoint(HLP,new_goal,agent_info,SLP_lookahead_distance); % new_goal; % 
                        else 
                            waypoint = new_goal;
                            % update current waypoints
                            HLP.current_waypoint = waypoint;
                            HLP.waypoints = [HLP.waypoints, waypoint];
                            HLP.current_waypoint_index = HLP.current_waypoint_index + 1;
                        end
                    else % choose next waypoint
                        if HLP.current_graph_waypoint_index == size(HLP.graph_waypoints,2) % (:,end) % if already at last waypoint, return goal
    %                         waypoint = get_SLP_to_waypoint(HLP,HLP.goal,agent_info,lookahead_distance);
                            waypoint = HLP.goal;
                            HLP.current_waypoint_index = length(HLP.waypoints);
                            HLP.nearly_reach_goal = true;
                        else % return the next waypoint
                            HLP.current_graph_waypoint_index = HLP.current_graph_waypoint_index + 1;
                            new_goal = HLP.graph_waypoints(:,HLP.current_graph_waypoint_index);
                            if use_SLP
                                waypoint = get_SLP_to_waypoint(HLP,new_goal,agent_info,SLP_lookahead_distance); % new_goal; % 
                            else 
                                waypoint = new_goal;
                                % update current waypoints
                                HLP.current_waypoint = waypoint;
                                HLP.waypoints = [HLP.waypoints, waypoint];
                                HLP.current_waypoint_index = HLP.current_waypoint_index + 1;
                            end
    %                         HLP.current_waypoint = waypoint;
                        end
                        HLP.waypoint_counter = 0;
                    end
                else % tracked one graph waypoint for too long
                    % can be triggered by goal being tracked too long but
                    % then just keep passing back goal?
                    if HLP.current_graph_waypoint_index == size(HLP.graph_waypoints,2) % (:,end) % if already at last waypoint, return goal
%                         waypoint = get_SLP_to_waypoint(HLP,HLP.goal,agent_info,lookahead_distance);
                        waypoint = HLP.goal;
                        HLP.current_waypoint_index = length(HLP.waypoints);
                        HLP.nearly_reach_goal = true;
                    else % return the next waypoint
                        HLP.current_graph_waypoint_index = HLP.current_graph_waypoint_index + 1;
                        new_goal = HLP.graph_waypoints(:,HLP.current_graph_waypoint_index);
                        if use_SLP
                            waypoint = get_SLP_to_waypoint(HLP,new_goal,agent_info,SLP_lookahead_distance); % new_goal; % 
                        else 
                            waypoint = new_goal;
                            % update current waypoints
                            HLP.current_waypoint = waypoint;
                            HLP.waypoints = [HLP.waypoints, waypoint];
                            HLP.current_waypoint_index = HLP.current_waypoint_index + 1;
                        end
%                         HLP.current_waypoint = waypoint;
                    end
                    HLP.waypoint_counter = 0;
                end
            end

            fprintf('        HLP: Current waypoint index: %d out of %d\n', HLP.current_graph_waypoint_index, size(HLP.graph_waypoints,2));

%             HLP.current_waypoint_patch_data = agent_info.get_collision_check_volume(waypoint) ;
            HLP.current_waypoint_patch_data = agent_info.get_collision_check_volume(HLP.graph_waypoints(:,HLP.current_graph_waypoint_index)) ;
        end

        function waypoint = get_SLP_to_waypoint(HLP,goal,agent_info,lookahead_distance)
            if nargin < 4
                lookahead_distance = HLP.default_lookahead_distance;
            end
    
            g = goal; % HLP.graph_waypoints(:,HLP.current_graph_waypoint_index);
            z = agent_info.state(HLP.arm_joint_state_indices, end);
            dir_des = angdiff(z, g);
            dir_des = dir_des./norm(dir_des);
            
            % adjust lookahead distance to not overshoot goal
            dist_to_goal = norm(angdiff(z, g));
            if lookahead_distance > dist_to_goal
                lookahead_distance = dist_to_goal;
            end
            
            waypoint = lookahead_distance.*dir_des + z;
    
            % update current waypoints
            HLP.current_waypoint = waypoint;
            HLP.waypoints = [HLP.waypoints, waypoint];
            HLP.current_waypoint_index = HLP.current_waypoint_index + 1;
        end
    end
end