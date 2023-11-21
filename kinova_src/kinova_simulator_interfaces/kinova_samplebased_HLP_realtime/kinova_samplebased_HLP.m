classdef kinova_samplebased_HLP < robot_arm_graph_planner_HLP
    properties
        sample_nodes = [];
        start_goal_distance_threshold = norm(pi / 48 * ones(1,7));
    end

    methods
        function HLP = kinova_samplebased_HLP(varargin)
            HLP@robot_arm_graph_planner_HLP(varargin{:}) ;
%             HLP.sample_nodes = load('millionNodes.mat');
            
            % Different sets of nodes
            % Uniformly sampled nodes across workspace: uniformNodes.mat
            % Composite set of nodes: QConfig_composite.txt
            % Composite set of nodes for hardare: QConfig_composite_hardware_only.txt
            sample_nodes = load('../kinova_simulator_interfaces/kinova_samplebased_HLP_realtime/uniformNodes.csv');            
            % check if proper structure HLP.sample_nodes.q_valid_list
            if class(sample_nodes) == 'double'
                sample_nodes(:,1) = wrapToPi(sample_nodes(:,1));
                HLP.sample_nodes.q_valid_list = sample_nodes;
            else
                sample_nodes.q_valid_list(:,1) = wrapToPi(sample_nodes.q_valid_list(:,1));
                HLP.sample_nodes = sample_nodes;
            end
            
            % code expects sample nodes to be 7 x n array (where n>7)
            if size(HLP.sample_nodes.q_valid_list,2) < size(HLP.sample_nodes.q_valid_list,1)
                HLP.sample_nodes.q_valid_list = HLP.sample_nodes.q_valid_list';
            end

        end

        function HLP = generatePath(HLP, obstacles, start, goal)
            % obstacle number hardcoded as 10
            Zs = [];
            for i = 1:10
                Zs = [Zs; obstacles{i}.Z'];
            end
            
            % write obstacle info to file as the input of the CUDA collision checker
            writematrix(Zs, '../kinova_simulator_interfaces/kinova_samplebased_HLP_realtime/obstacles.csv', 'Delimiter', ' ');
            
            % call collision checker in CUDA
            system('./../kinova_simulator_interfaces/kinova_samplebased_HLP_realtime/collision_checker');
            
            tic;
            adj_matrix_sparse_data = readmatrix('../kinova_simulator_interfaces/kinova_samplebased_HLP_realtime/collision_free_adj_matrix.csv');
            adj_matrix_sparse = sparse(adj_matrix_sparse_data(:,1)+1, ...
                                       adj_matrix_sparse_data(:,2)+1, ...
                                       adj_matrix_sparse_data(:,3), ...
                                       size(HLP.sample_nodes.q_valid_list,2), size(HLP.sample_nodes.q_valid_list,2));
            G = graph(adj_matrix_sparse, 'lower');
            % add configuration information to graph?
            
            [bins, binsize] = conncomp(G);
            [~, max_id] = max(binsize);
            G_maxconn = subgraph(G, bins == max_id);
            build_graph_time = toc;
            fprintf('    HLP: Building graph takes %f\n', build_graph_time)
            
            q_subgraph = HLP.sample_nodes.q_valid_list(:, bins == max_id);
            difference_to_goal = vecnorm(wrapToPi(goal - q_subgraph));
            difference_to_start = vecnorm(wrapToPi(start - q_subgraph));
            [end_diff, end_idx] = min(difference_to_goal);
            [start_diff, start_idx] = min(difference_to_start);

            if end_diff > HLP.start_goal_distance_threshold
                fprintf('    HLP: Goal node is far away!!! Distance: %f\n', end_diff)
            end
            if start_diff > HLP.start_goal_distance_threshold
                fprintf('    HLP: Start node is far away!!! Distance: %f\n', start_diff)
            end
            
            [path, len] = shortestpath(G_maxconn, start_idx, end_idx);
%             [path, len] = shortestpath(G_maxconn, start_node, goal_node);
            
            if isempty(path)
                error('can not find any path!');
            end

            HLP.graph_waypoints = q_subgraph(:,path);
%             HLP.graph_waypoints = G(:,path);
%             test = 1;
            % adding noise to waypoints
%             for i = 1:size(HLP.graph_waypoints,2)
%                 HLP.graph_waypoints(5:7,i) = HLP.graph_waypoints(5:7,i) + (rand(3,1)-1/2)*2*0.3;
%             end
%             HLP.graph_waypoints = [-2.51327412287183	-2.51327412287183	-2.51327412287183	-2.51327412287183	-2.82743338823081	-3.14159265358979	2.82743338823081	2.51327412287183	2.19911485751286	1.88495559215388	1.88495559215388;
%             -1.14157894736842	-0.887894736842105	-0.634210526315789	-0.380526315789474	-0.380526315789474	-0.380526315789474	-0.380526315789474	-0.380526315789474	-0.380526315789474	-0.380526315789474	-0.634210526315789;
%             0	0	0	0	0	0	0	0	0	0	0;
%             -2.10000000000000	-2.10000000000000	-2.10000000000000	-2.10000000000000	-2.10000000000000	-2.10000000000000	-2.10000000000000	-2.10000000000000	-2.10000000000000	-2.10000000000000	-2.10000000000000;
%             0.0718779512175262	0.0623789646291634	-0.0593903472973539	0.237472440918607	-0.285363896843216	-0.281021052826672	0.284410289150401	-0.0841749379057776	0.173067202801137	0.126590345549042	0.275867105667405;
%             1.56549415918561	1.38678682793423	1.22161979894148	0.810375570176046	0.936752858783413	0.747574501909254	1.06484695382426	0.864267033398110	0.806718598896218	0.893758605177782	1.26147440545500;
%             -0.229176976385724	0.151978950627273	0.244189126835185	-0.264858073509244	0.252976179478014	-0.0527516713774990	0.0395271969927084	-0.237711992165667	0.112245829003490	0.200002718388178	0.135900280148327];
        end

        function plot(HLP)
            % plot the waypoint
            if ~isempty(HLP.current_waypoint_patch_data)
%                 if ~isempty(HLP.plot_data.waypoint_arm_volume)
%                     delete(HLP.plot_data.waypoint_arm_volume)
%                 end
                HLP.plot_data.waypoint_arm_volume = patch(HLP.current_waypoint_patch_data,...
                    'FaceColor','c','FaceAlpha',0.1,...
                    'EdgeColor','c','EdgeAlpha',0.5) ;
%             if ~isempty(HLP.previous_waypoint_patch_data)
% %                 if ~isempty(HLP.plot_data.previous_waypoint_arm_volume)
% %                     delete(HLP.plot_data.previous_waypoint_arm_volume)
% %                 end
%                 HLP.plot_data.previous_waypoint_arm_volume = patch(HLP.previous_waypoint_patch_data,...
%                     'FaceColor','b','FaceAlpha',0.1,...
%                     'EdgeColor','b','EdgeAlpha',0.5) ;
%             end
%             if ~isempty(HLP.next_waypoint_patch_data)
% %                 if ~isempty(HLP.plot_data.next_waypoint_arm_volume)
% %                     delete(HLP.plot_data.next_waypoint_arm_volume)
% %                 end
%                 HLP.plot_data.next_waypoint_arm_volume = patch(HLP.next_waypoint_patch_data,...
%                     'FaceColor','w','FaceAlpha',0.1,...
%                     'EdgeColor','w','EdgeAlpha',0.5) ;
%             end
            end
        end
    end
end



