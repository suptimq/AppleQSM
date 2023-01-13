function [] = segmentation(data_folder, skel_folder, tree_id, exp_id, options)

    % plot graph prior to MST
    DEBUG = options.DEBUG;
    % logging
    LOGGING = options.LOGGING;
    % skeleton refinement
    TRUNK_REFINEMENT = options.TRUNK_REFINEMENT;
    BRANCH_REFINEMENT = options.BRANCH_REFINEMENT;
    % save parameters for each tree
    SAVE_PARAS = options.SAVE_PARAS;
    LOAD_PARAS = options.LOAD_PARAS;
    % plot and save figures
    SAVE_FIG = options.SAVE_FIG;
    SHOW_CLUSTER_SPLIT = options.SHOW_CLUSTER_SPLIT;

    skel_filename_format = '_contract_*_skeleton.mat';
    skel_filename = search_skeleton_file(tree_id, skel_folder, skel_filename_format);
    paras_filename = [exp_id '_parameters.mat'];

    branch_folder = fullfile(data_folder, [tree_id '_branch']);
    files = dir(fullfile(branch_folder, 'Section*.ply'));

    output_folder = fullfile(skel_folder, '..', 'segmentation', exp_id);
    log_filepath = fullfile(output_folder, [tree_id '_log']);
    paras_filepath = fullfile(output_folder, paras_filename);
    paras = struct();

    if LOGGING
        diary logfile
    end

    %% create folder to save results
    if ~exist(output_folder, 'dir')
        mkdir(output_folder)
    end

    %% load data
    skel_filepath = fullfile(skel_folder, skel_filename);
    new_skel_filepath = fullfile(output_folder, skel_filename);
    load(skel_filepath, 'P'); % P results from skeleton operation

    if exist(paras_filepath, 'file') && LOAD_PARAS
        load(paras_filepath, 'paras');
    end

    % visualization purpose only and show the original point clouds
    original_pt_normalized = P.original_pt;
    original_pt_normalized_location = original_pt_normalized.Location;
    desired_pt = 10000;
    ratio = desired_pt / original_pt_normalized.Count;
    pt = pcdownsample(original_pt_normalized, 'random', ratio); % visualization purpose only!

    if SAVE_FIG
        filename = fullfile(output_folder, 'original_point_cloud');
        figure('name', 'Original point cloud')
        pcshow(pt, 'markersize', 30);
        hold on
        axis equal;
        axis off;
        view(0, -5);
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    %% show skeleton connectivity
    figure('Name', 'Skeleton connectivity');
    set(gcf, 'color', 'white')
    sizep = 100; sizee = 1; colorp = [1, .8, .8]; colore = [1, .0, .0];
    scatter3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), sizep, '.'); hold on; axis equal;
    set(gcf, 'Renderer', 'OpenGL');
    plot_connectivity(P.spls, P.spls_adj, sizee, colore);

    if SAVE_FIG
        filename = fullfile(output_folder, 'skeleton_connectivity');
        % saveas(gcf, filename);
        axis off;
        view(0, -5);
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    %% show skeleton and original point cloud
    figure('Name', 'Original point cloud and its skeleton');
    pcshow(pt, 'markersize', 30); hold on;
    set(gcf, 'color', 'white'); set(gca, 'color', 'white');
    plot3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), '.r', 'markersize', 20);
    axis equal;

    if SAVE_FIG
        filename = fullfile(output_folder, 'skeleton_overlaid');
        saveas(gcf, filename);
        axis off;
        view(0, -5);
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    %%---------------------------------------------------------%%
    %%-------------------Trunk Identification-------------------%%
    %% identify tree trunk by developing a MST and
    %% find the path with maximum weight
    %%---------------------------------------------------------%%
    disp('===================Trunk Identification===================');

    %% create a graph with density as weights
    distance_th = load_parameters(paras, 'distance_th_lambda1', 0.1);
    mode = load_parameters(paras, 'entire_graph_refine_mode', 'distance');
    coefficient_inv_density_weight = load_parameters(paras, 'graph_edge_coefficient_alpha1', 0.6);
    [adj_matrix, adj_idx, density_weight, inv_density_weight, distance_weight] = refine_adj_matrix(P.spls, P.spls_adj, P.spls_density, distance_th, mode);
    % normalize inv_density_weight and distance_weight to [0, 1]
    inv_weight_normalized = normalize(inv_density_weight, 'range');
    distance_weight_normalized = normalize(distance_weight, 'range');
    graph_weight = coefficient_inv_density_weight * inv_weight_normalized + (1 - coefficient_inv_density_weight) * distance_weight_normalized;
    inverse_weighted_graph = graph(adj_idx(1, :), adj_idx(2, :), graph_weight);

    %%
    if DEBUG
        figure('Name', 'Refined skeleton connectivity')
        colore = [1, .0, .0]; sizee = 2;
        pcshow(P.spls, 'markersize', 80); hold on;
        plot_connectivity(P.spls, adj_matrix, sizee, colore);
        axis equal;
    end

    % plot graph
    figure('Name', 'Graph')
    plot_weighted_graph = plot(inverse_weighted_graph);
    hold on

    % select root point candidates
    root_point_search_range = load_parameters(paras, 'root_point_search_range', P.sample_radius);
    [root_val, ~] = min(P.spls(:, 3));
    root_points_idx = find(P.spls(:, 3) < root_val + root_point_search_range);
    root_points = P.spls(root_points_idx, :);

    % compute the most spreadout MST
    MSTs_length = zeros(length(root_points_idx), 1);

    for i = 1:size(root_points, 1)
        root_point_idx = root_points_idx(i);
        [MST, ~] = minspantree(inverse_weighted_graph, 'Type', 'tree', 'Root', findnode(inverse_weighted_graph, root_point_idx));
        MST_length = size(MST.Edges, 1);
        MSTs_length(i) = MST_length;
        highlight(plot_weighted_graph, MST, 'NodeColor', 'g');
        highlight(plot_weighted_graph, findnode(inverse_weighted_graph, root_point_idx), 'NodeColor', 'r', 'MarkerSize', 5);
    end

    if SAVE_FIG
        filename = fullfile(output_folder, 'weighted_graph');
        saveas(gcf, filename);
        axis off;
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    [MST_max, ~] = max(MSTs_length);
    MST_max_idxs = MSTs_length(:) == MST_max;
    [~, MST_max_idx] = min(P.spls(root_points_idx(MST_max_idxs), 3));

    root_point_idx_max_MST = root_points_idx(MST_max_idx);
    root_point_max_MST = root_points(MST_max_idx, :);

    [MST, ~] = minspantree(inverse_weighted_graph, 'Type', 'tree', 'Root', findnode(inverse_weighted_graph, root_point_idx_max_MST));
    MST_nodes = table2array(MST.Edges);
    MST_nodes_unique = unique(MST_nodes(:, 1:2));
    MST_pts = P.spls(MST_nodes_unique, :);

    %% plot connectivity for MST
    MST_adj_matrix = zeros(length(P.spls), length(P.spls));

    for i = 1:size(MST_nodes, 1)
        MST_adj_matrix(MST_nodes(i, 1), MST_nodes(i, 2)) = 1;
    end

    figure('Name', 'MST connectivity')
    scatter3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), sizep, '.'); hold on;
    plot_connectivity(P.spls, MST_adj_matrix, sizee, colore);
    axis equal;

    if SAVE_FIG
        filename = fullfile(output_folder, 'MST_connectivity');
        saveas(gcf, filename);
        axis off;
        view(0, -5);
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    %% compute the edge weights from each MST node to the root point
    % main trunk end point is the one with the maximum path density_weight

    % change MST weights from inverse density to density
    adj_idx_trans = adj_idx'; % transpose to Nx2 for 'ismember'
    [lia, ~] = ismember(adj_idx_trans, MST_nodes(:, 1:2), 'row');
    density_weight_MST = density_weight(lia == 1, :);
    distance_weight_MST = distance_weight(lia == 1, :);
    density_weight_MST_normalized = normalize(density_weight_MST, 'range');
    distance_weight_MST_normalized = normalize(distance_weight_MST, 'range');
    coefficient_density_weight = load_parameters(paras, 'graph_edge_coefficient_alpha2', 0.4);
    MST_weight = coefficient_density_weight * density_weight_MST_normalized + (1 - coefficient_density_weight) * distance_weight_MST_normalized;
    MST.Edges.Weight = MST_weight;
    % MST.Edges.Weight = (density_weight(lia == 1, :) + 1).^2; % scale weights to enlarge the gap
    % MST.Edges.Weight = density_weight(lia == 1, :); % scale weights to enlarge the gap

    %% shortest path to detect main trunk
    shortest_path_nodes = cell(size(P.spls, 1), 1);
    shortest_path_distance = zeros(size(P.spls, 1), 1);

    for i = 1:length(P.spls)
        [node, distance] = shortestpath(MST, root_point_idx_max_MST, i);

        if isempty(node)
            distance = 0; % avoid inf
        end

        shortest_path_nodes{i} = node;
        shortest_path_distance(i) = distance;
    end

    density_search_range = load_parameters(paras, 'density_search_range', 0.1);
    [max_density, ~] = max(shortest_path_distance);
    endpoint_pts_idx = find(shortest_path_distance >= max_density - density_search_range);

    endpoint_pts = P.spls(endpoint_pts_idx, :);
    [highest_pts_z, highest_pts_idx] = max(endpoint_pts(:, 3));

    max_density_idx = endpoint_pts_idx(highest_pts_idx);
    main_trunk_endpoint = P.spls(max_density_idx, :);
    main_trunk_pts_idx = shortest_path_nodes{max_density_idx}'; % take care of dimension!
    main_trunk_pts = P.spls(main_trunk_pts_idx, :);
    main_trunk_pts = sortrows(main_trunk_pts, 3); % sort by height
    [~, main_trunk_pts_idx] = ismember(main_trunk_pts, P.spls, 'row');

    figure('Name', 'Coarse main trunk')
    pcshow(P.spls, 'markersize', 80); hold on
    plot3(root_point_max_MST(1), root_point_max_MST(2), root_point_max_MST(3), 'r.', 'markersize', 20);
    plot3(main_trunk_pts(:, 1), main_trunk_pts(:, 2), main_trunk_pts(:, 3), '.r', 'markersize', 15);
    plot3(main_trunk_endpoint(1), main_trunk_endpoint(2), main_trunk_endpoint(3), 'b.', 'markersize', 20);
    axis equal;

    if SAVE_FIG
        filename = fullfile(output_folder, 'coarse_main_trunk');
        saveas(gcf, filename);
        axis off;
        % view(0, -5);
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    disp(['distance threshold lambda1: ' num2str(distance_th)]);
    disp(['entire graph refine mode: ' mode]);
    disp(['graph edge coefficient alpha1: ' num2str(coefficient_inv_density_weight)]);
    disp(['root point search range: ' num2str(root_point_search_range)]);

    disp(['graph edge coefficient alpha2: ' num2str(coefficient_density_weight)]);
    disp(['density search range: ' num2str(density_search_range)]);

    %%---------------------------------------------------------%%
    %%-------------------Trunk Diameter Est.-------------------%%
    %% compute trunk diameter using least square fitting
    %% find the index of the root point in the original point cloud
    %%---------------------------------------------------------%%
    disp('===================Trunk Diameter Estimation===================');

    loc = backproject(root_point_max_MST, P);
    ori_root_point = P.pts(loc, :);
    slice_range_z_axis = 0.008; % 8mm
    fitting_pts_idx = P.pts(:, 3) <= ori_root_point(:, 3) + slice_range_z_axis;
    fitting_pts = P.pts(fitting_pts_idx, :);
    min_samples = load_parameters(paras, 'ransac_trunk_diameter_min_sample', 30);
    residual_threshold = load_parameters(paras, 'ransac_trunk_diameter_threshold', 0.005);
    max_trials = load_parameters(paras, 'ransac_trunk_diameter_trials', 100);

    [ellipse, inliers, outliers] = ransac_py(fitting_pts(:, 1:2), 'Ellipse', min_samples, residual_threshold, max_trials);
    xc = ellipse(1); yc = ellipse(2); radius_x = ellipse(3); radius_y = ellipse(4); theta = ellipse(5); trunk_radius = (radius_x + radius_y) / 2;

    inliers_idx = find(inliers == 1);
    outliers_idx = find(outliers == 1);

    %% visualization of main trunk diameter fitting
    figure('Name', 'Ellipse fitting')
    scatter(fitting_pts(inliers_idx, 1), fitting_pts(inliers_idx, 2), '.', 'b'); hold on
    scatter(fitting_pts(outliers_idx, 1), fitting_pts(outliers_idx, 2), '.', 'r');
    draw_ellipse(radius_x, radius_y, theta, xc, yc, 'k')

    if SAVE_FIG
        filename = fullfile(output_folder, 'diameter_ellipse_fitting');
        saveas(gcf, filename);
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    figure('Name', 'Ellipse from RANSAC')
    subplot(1, 2, 1)
    scatter(fitting_pts(inliers_idx, 1), fitting_pts(inliers_idx, 2), '.', 'r'); hold on
    scatter(fitting_pts(outliers_idx, 1), fitting_pts(outliers_idx, 2), '.', 'b');
    title('Original points in XY plane')

    subplot(1, 2, 2)
    scatter(fitting_pts(inliers_idx, 1), fitting_pts(inliers_idx, 2), '.', 'r'); hold on
    draw_ellipse(radius_x, radius_y, theta, xc, yc, 'k')
    title(['Radius: x-axis ', num2str(radius_x * 1e3, '%.1f'), ' y-axis: ', num2str(radius_y * 1e3, '%.1f'), ' avg: ', num2str(trunk_radius * 1e3, '%.1f'), ' mm'], 'color', [1, 0, 0]);

    if SAVE_FIG
        filename = fullfile(output_folder, 'diameter_ellipse_fitting');
        saveas(gcf, filename);
    end

    disp(['ransac trunk diameter min sample: ' num2str(min_samples)]);
    disp(['ransac trunk diameter threshold: ' num2str(residual_threshold)]);
    disp(['ransac trunk diameter max trials: ' num2str(max_trials)]);

    if TRUNK_REFINEMENT
        %%---------------------------------------------------%%
        %%-------------------Trunk CPC-------------------%%
        %%---------------------------------------------------%%
        disp('===================Trunk Skeleton Refinement===================');

        trunk_refinement_options.xy_radius = max(radius_x, radius_y);
        trunk_refinement_options.z_radius = 0.02;
        trunk_refinement_options.maximum_length = 0.005;
        trunk_refinement_options.N = 10;
        trunk_refinement_options.M = 100;
        [trunk_cpc_optimized_center, trunk_cpc_optimized_radius, trunk_cpc_optimized_confidence, trunk_pc] = trunk_refinement(P, main_trunk_pts_idx, trunk_refinement_options);
        P.trunk_cpc_optimized_center = trunk_cpc_optimized_center;
        P.trunk_cpc_optimized_radius = trunk_cpc_optimized_radius;
        P.trunk_cpc_optimized_confidence = trunk_cpc_optimized_confidence;
        P.trunk_pc = trunk_pc;
        P.trunk_radius = trunk_radius;
        save(new_skel_filepath, 'P');

        disp(['xy radius: ' num2str(trunk_refinement_options.xy_radius)]);
        disp(['z radius: ' num2str(trunk_refinement_options.z_radius)]);
        disp(['maximum length: ' num2str(trunk_refinement_options.maximum_length)]);
        disp(['N: ' num2str(trunk_refinement_options.N)]);
        disp(['M: ' num2str(trunk_refinement_options.M)]);
        disp('===================Finish Trunk Refinement by CPC and Saved===================');

        figure('Name', 'Optimized skeleton pts')
        ax1 = subplot(1, 2, 1);
        pcshow(trunk_pc, 'MarkerSize', 30); hold on
        plot3(trunk_cpc_optimized_center(:, 1), trunk_cpc_optimized_center(:, 2), trunk_cpc_optimized_center(:, 3), '.r', 'MarkerSize', 30)
        title('Skeleton pts', 'color', [1, 0, 0])
        xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

        ax2 = subplot(1, 2, 2);
        plot_by_weight(trunk_cpc_optimized_center, trunk_cpc_optimized_radius / trunk_radius)
        title('Skeleton pts radius', 'color', [1, 0, 0])
        xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

        Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
        setappdata(gcf, 'StoreTheLink', Link);
    end

    distance_list = [];
    refined_main_trunk_pts = P.trunk_cpc_optimized_center;

    for i = 1:size(refined_main_trunk_pts, 1) - 1
        distance_ = pdist([refined_main_trunk_pts(i); refined_main_trunk_pts(i + 1)]);
        distance_list = [distance_list, distance_];
    end

    ratio_distance_list = [0, cumsum(distance_list) / sum(distance_list)];
    P.main_trunk_length = sum(distance_list);
    P.trunk_internode_distance_ratio = ratio_distance_list;

    % main trunk refinement (heritage issue)
    disp('===================Trunk Skeleton Refinement (Heritage)===================');
    P.main_trunk_height = main_trunk_endpoint(3) - min(P.spls(:, 3)); % height before normalization
    main_trunk_refine_range = load_parameters(paras, 'main_trunk_refine_range', P.sample_radius);
    new_main_trunk_pts_idx = zeros(0, 1);

    for i = 1:length(main_trunk_pts_idx) - 1
        v1 = P.spls(main_trunk_pts_idx(i), :);
        v2 = P.spls(main_trunk_pts_idx(i + 1), :);

        v1_z = v1(3);
        v2_z = v2(3);
        pts_idx = find(P.spls(:, 3) >= v1_z & P.spls(:, 3) <= v2_z);

        for j = 1:size(pts_idx, 1)
            cur_pt = P.spls(pts_idx(j), :);
            distance = point_to_line_distance(cur_pt, v1, v2);

            if distance < main_trunk_refine_range
                new_main_trunk_pts_idx(end + 1, 1) = pts_idx(j);
            end

        end

    end

    refined_main_trunk_pts_idx = [main_trunk_pts_idx; new_main_trunk_pts_idx];
    refined_main_trunk_pts_idx = unique(refined_main_trunk_pts_idx);
    refined_main_trunk_pts = P.spls(refined_main_trunk_pts_idx, :);
    refined_main_trunk_pts = sortrows(refined_main_trunk_pts, 3);
    [~, refined_main_trunk_pts_idx] = ismember(refined_main_trunk_pts, P.spls, 'row');
    rest_pts_idx = setdiff(1:size(P.spls, 1), refined_main_trunk_pts_idx);
    rest_pts = P.spls(rest_pts_idx, :);

    figure('Name', 'Refined main trunk')
    pcshow(P.spls, 'markersize', 50); hold on
    plot3(root_point_max_MST(1), root_point_max_MST(2), root_point_max_MST(3), 'r.', 'markersize', 20);
    plot3(refined_main_trunk_pts(:, 1), refined_main_trunk_pts(:, 2), refined_main_trunk_pts(:, 3), '.r', 'markersize', 15);
    plot3(main_trunk_endpoint(1), main_trunk_endpoint(2), main_trunk_endpoint(3), 'r.', 'markersize', 20);
    axis equal;

    if SAVE_FIG
        filename = fullfile(output_folder, 'refine_main_trunk');
        saveas(gcf, filename);
        axis off;
        % view(0, -5);
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    figure('Name', 'Refined main trunk and branch')
    subplot(1, 2, 1)
    pcshow(P.spls, 'markersize', 50); hold on
    plot3(refined_main_trunk_pts(:, 1), refined_main_trunk_pts(:, 2), refined_main_trunk_pts(:, 3), '.r', 'markersize', 15);
    title(['Height: ', num2str(P.main_trunk_height * 100, '%.0f'), ' cm'], 'color', [1, 0, 0]);
    axis equal;

    subplot(1, 2, 2)
    pcshow(P.spls, 'markersize', 50); hold on
    plot3(rest_pts(:, 1), rest_pts(:, 2), rest_pts(:, 3), '.r', 'markersize', 15);
    title(['Length: ', num2str(P.main_trunk_length * 100, '%.0f'), ' cm'], 'color', [1, 0, 0]);
    axis equal;

    if SAVE_FIG
        filename = fullfile(output_folder, 'trunk_branch');
        saveas(gcf, filename);
    end

    disp(['main trunk refine range: ' num2str(main_trunk_refine_range)]);

    %%---------------------------------------------------------%%
    %%-------------------Branch Root Cluster-------------------%%
    %% sphere pruning centering at trunk points
    %% run clustering on pruned branch root points
    %%---------------------------------------------------------%%
    %% pre-filter branch root points in crotch area
    disp('===================Running DBSCAN on crotch points===================');
    refined_main_trunk_pts = P.trunk_cpc_optimized_center;
    sphere_radius = 6 * P.sample_radius;
    crotch_pts_index = sphere_pruning(P, refined_main_trunk_pts_idx, sphere_radius);
    crotch_pts = P.spls(crotch_pts_index, :);
    crotch_pts = sortrows(crotch_pts, 3);
    disp('===================Sort crotch points by Z-axis===================');
    disp(['sphere pruning radius: ' num2str(sphere_radius)]);

    %% DBSCAN clustering
    noise_label = -1;
    eps = load_parameters(paras, 'branch_seg_dbscan_eps', P.sample_radius * 1.5);
    min_samples = load_parameters(paras, 'branch_seg_dbscan_min_samples', 3);
    cluster_label = dbscan(crotch_pts, eps, min_samples);
    noise_pts = crotch_pts(cluster_label == noise_label, :);
    crotch_pts = crotch_pts(cluster_label ~= noise_label, :);
    cluster_label = cluster_label(cluster_label ~= noise_label);
    unique_cluster_label = unique(cluster_label);
    disp(['dbscan eps: ' num2str(eps)]);
    disp(['dbscan min samples: ' num2str(min_samples)]);

    figure('Name', 'Results from Clustering')
    ax1 = subplot(1, 3, 1);
    plot3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), '.g', 'Markersize', 20); hold on
    plot3(refined_main_trunk_pts(:, 1), refined_main_trunk_pts(:, 2), refined_main_trunk_pts(:, 3), '.r', 'markersize', 30);
    plot3(crotch_pts(:, 1), crotch_pts(:, 2), crotch_pts(:, 3), '.b', 'markersize', 30);
    title('Crotch points w/ raw trunk skeleton points', 'color', [1, 0, 0]);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    ax2 = subplot(1, 3, 2);
    plot3(P.trunk_cpc_optimized_center(:, 1), P.trunk_cpc_optimized_center(:, 2), P.trunk_cpc_optimized_center(:, 3), '.r', 'markersize', 30); hold on
    plot3(rest_pts(:, 1), rest_pts(:, 2), rest_pts(:, 3), '.g', 'Markersize', 30);
    plot3(noise_pts(:, 1), noise_pts(:, 2), noise_pts(:, 3), '.yellow', 'markersize', 30);
    plot3(crotch_pts(:, 1), crotch_pts(:, 2), crotch_pts(:, 3), '.b', 'markersize', 30);
    title('Crotch points w/ uniform trunk skeleton points', 'color', [1, 0, 0]);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    ax3 = subplot(1, 3, 3);
    plot3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), '.g', 'Markersize', 20); hold on
    plot3(refined_main_trunk_pts(:, 1), refined_main_trunk_pts(:, 2), refined_main_trunk_pts(:, 3), '.black', 'markersize', 30);
    plot_dbscan_clusters(crotch_pts, cluster_label);
    title(['Clusters: ', num2str(length(unique_cluster_label))], 'color', [1, 0, 0]);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    Link = linkprop([ax1, ax2, ax3], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
    setappdata(gcf, 'StoreTheLink', Link);

    figure('Name', '1st DBSCAN clusters')
    ax1 = subplot(1, 2, 1);
    pcshow(original_pt_normalized, 'MarkerSize', 30); hold on
    plot3(P.trunk_cpc_optimized_center(:, 1), P.trunk_cpc_optimized_center(:, 2), P.trunk_cpc_optimized_center(:, 3), '.r', 'markersize', 30);
    plot3(crotch_pts(:, 1), crotch_pts(:, 2), crotch_pts(:, 3), '.b', 'markersize', 30);
    title('Crotch points')
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    ax2 = subplot(1, 2, 2);
    pcshow(original_pt_normalized, 'markersize', 40); hold on
    plot_dbscan_clusters(crotch_pts, cluster_label);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;
    title(['Initial clusters: ', num2str(length(unique_cluster_label))], 'color', [1, 0, 0]);

    Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
    setappdata(gcf, 'StoreTheLink', Link);

    %%----------------------------------------------------------%%
    %%-------------------Post-Process Cluster-------------------%%
    %% split under-segmented clusters
    %% 1. check if the cluster contains multiple branches
    %% 2. find the critical point and split multiple branches
    %% intrisically, sort branch points by their distance to trunk
    %%----------------------------------------------------------%%
    cluster_counter = 0;
    unique_updated_cluster_label = [];
    updated_cluster_label = [];
    updated_cluster_pts_list = [];
    updated_cluster_pts_cell = {};

    for i = 1:length(unique_cluster_label)
        cur_cluster_label = unique_cluster_label(i);

        cur_cluster_pts_idx = cluster_label == cur_cluster_label;
        cur_pts = crotch_pts(cur_cluster_pts_idx, :);
        [sliced_main_trunk_pts, row, col] = find_internode(double(cur_pts), refined_main_trunk_pts, 0.1, false);
        trunk_internode = sliced_main_trunk_pts(row, :);
        branch_internode = cur_pts(col, :);

        % fit a 3D line representing the sliced main trunk
        % the main trunk pts used to fit the line shouldn't be too long
        if size(sliced_main_trunk_pts, 1) - row < 5
            tmp_ii = 1:size(sliced_main_trunk_pts, 1);
        else
            tmp_ii = 1:row + 5;
        end

        min_samples = 3; residual_threshold = 0.005; max_trials = 1e3;
        [sliced_vector, ~, ~] = ransac_py(sliced_main_trunk_pts(tmp_ii, :), '3D_Line', min_samples, residual_threshold, max_trials);

        v1 = sliced_vector(1:3);
        v2 = sliced_vector(1:3) + sliced_vector(4:6);
        trunk_vector_branch_distance_matrix = point_to_line_distance(cur_pts, v1, v2);
        [~, trunk_internode_branch_distance_index] = mink(trunk_vector_branch_distance_matrix, size(cur_pts, 1));
        branch_branch_distance_matrix = pdist2(double(cur_pts), double(cur_pts));

        if SHOW_CLUSTER_SPLIT
            figure('Name', ['Cluster ', num2str(i)])
            subplot(1, 2, 1)
            plot3(sliced_main_trunk_pts(:, 1), sliced_main_trunk_pts(:, 2), sliced_main_trunk_pts(:, 3), '.r', 'MarkerSize', 30); hold on
            plot3(trunk_internode(1), trunk_internode(2), trunk_internode(3), '.black', 'MarkerSize', 30);
            plot3(cur_pts(:, 1), cur_pts(:, 2), cur_pts(:, 3), '.b', 'MarkerSize', 30);
            plot3(branch_internode(1), branch_internode(2), branch_internode(3), '.black', 'MarkerSize', 30);
        end

        trunk_branch_distance_list = []; % distance between trunk internode and branch skeleton points
        branch_id_list = []; % branch id assigned by traverse order
        pts_list = []; % branch coordinates
        point_point_distance_list = {}; % point-to-point distance within each cluster
        point_trunk_gap_distance_list = {}; % point-to-trunk difference distance within each cluster
        %         neighbor_branch_id_list = [];                          % neighboring branch id in Queue
        critical_id_list = []; % ID of points whose distance changed suddenly

        tmp_trunk_branch_distance_list = [];
        tmp_original_branch_id_list = [];
        tmp_new_branch_id_list = [];
        tmp_pts_list = [];

        point_distance_list = []; % distance between neighboring points

        counter = 0;
        distance_threshold_multiplier = 3; % define maximum search range
        visited = zeros(size(cur_pts, 1), 1);
        queue = col;
        visited(col) = 1;

        while queue

            % pop the head element
            col = queue(1);
            queue(1) = [];
            tmp_original_branch_id_list = [tmp_original_branch_id_list, col];

            % temporarily storage
            counter = counter + 1;
            tmp_pts = cur_pts(col, :);
            tmp_distance = trunk_vector_branch_distance_matrix(col);

            tmp_trunk_branch_distance_list = [tmp_trunk_branch_distance_list, tmp_distance];
            tmp_new_branch_id_list = [tmp_new_branch_id_list, counter];
            tmp_pts_list = [tmp_pts_list; tmp_pts];

            if SHOW_CLUSTER_SPLIT
                text(tmp_pts(1), tmp_pts(2), tmp_pts(3) + 0.02, num2str(counter), 'Color', 'red', 'HorizontalAlignment', 'left', 'FontSize', 12);
            end

            % update point-to-point distance list
            % the distance between 1st branch point to trunk might be large due to the missing points in crotch
            if isempty(point_distance_list)
                %                     if tmp_distance > init_search_range
                %                         tmp_distance = init_search_range;
                %                     end
                point_distance_list = [point_distance_list, tmp_distance];
            else
                prev_original_branch_id = tmp_original_branch_id_list(counter - 1);
                cur_original_branch_id = tmp_original_branch_id_list(counter);
                tmp_branch_branch_distance = branch_branch_distance_matrix(prev_original_branch_id, cur_original_branch_id);
                point_distance_list = [point_distance_list, tmp_branch_branch_distance];
            end

            % find the next point that is closest to the current point - NOT the BEST WAY!!
            % find the next several point that are within the avg distance and save by a Queue
            cur_branch_distance_matrix = branch_branch_distance_matrix(col, :);
            [sort_cur_branch_distance_matrix, sort_index] = sort(cur_branch_distance_matrix);

            % search the neighboring points
            % search range becomes larger if no points are found
            tmp_multiplier = 0;
            add_queue_pts_index = 0;

            % the point itself is definitely included
            while sum(add_queue_pts_index) == 0 && tmp_multiplier < distance_threshold_multiplier - 1
                tmp_multiplier = tmp_multiplier + 1;
                neighbor_pts_index = sort_cur_branch_distance_matrix < mean(point_distance_list) * tmp_multiplier;
                neighbor_pts_index = sort_index(neighbor_pts_index);
                visited_pts_index = find(visited);
                add_queue_pts_index = setdiff(neighbor_pts_index, visited_pts_index, 'stable');
            end

            % if there are any neighbors within threshold, save to the queue
            % otherwise
            % if queue is empty, save results and find the next point that is closest to the trunk
            % if queue is not empty, continue process next element
            if sum(add_queue_pts_index) ~= 0
                % set visited and add index to queue
                visited(add_queue_pts_index) = 1;
                queue = [queue, add_queue_pts_index];
            end

            if isempty(queue)

                if length(tmp_new_branch_id_list) > 3

                    % average of distance |point1-to-trunk - point2-to-trunk|
                    tmp_list = [];

                    for j = 1:length(tmp_trunk_branch_distance_list) - 1
                        tmp_list = [tmp_list, abs(tmp_trunk_branch_distance_list(j) - tmp_trunk_branch_distance_list(j + 1))];
                    end

                    point_trunk_gap_distance_list{end + 1} = tmp_list;

                    trunk_branch_distance_list = [trunk_branch_distance_list, tmp_trunk_branch_distance_list];
                    branch_id_list = [branch_id_list, tmp_new_branch_id_list];
                    pts_list = [pts_list; tmp_pts_list];
                    point_point_distance_list{end + 1} = point_distance_list;
                    critical_id_list = [critical_id_list, counter];

                    % reset
                    tmp_trunk_branch_distance_list = [];
                    tmp_new_branch_id_list = [];
                    tmp_pts_list = [];
                    point_distance_list = [];

                end

                % find the next point that is closest to the trunk
                visited_pts_index = find(visited);
                add_queue_pts_index = setdiff(trunk_internode_branch_distance_index, visited_pts_index, 'stable');

                if ~isempty(add_queue_pts_index)
                    visited(add_queue_pts_index(1)) = 1;
                    queue = add_queue_pts_index(1);
                end

            end

        end

        if isempty(pts_list)
            % average of distance |point1-to-trunk - point2-to-trunk|
            tmp_list = [];

            for j = 1:length(tmp_trunk_branch_distance_list) - 1
                tmp_list = [tmp_list, abs(tmp_trunk_branch_distance_list(j) - tmp_trunk_branch_distance_list(j + 1))];
            end

            point_trunk_gap_distance_list{end + 1} = tmp_list;

            trunk_branch_distance_list = [trunk_branch_distance_list, tmp_trunk_branch_distance_list];
            branch_id_list = [branch_id_list, tmp_new_branch_id_list];
            pts_list = [pts_list; tmp_pts_list];
            point_point_distance_list{end + 1} = point_distance_list;
            critical_id_list = [critical_id_list, counter];

            % reset
            tmp_trunk_branch_distance_list = [];
            tmp_new_branch_id_list = [];
            tmp_pts_list = [];
            point_distance_list = [];

        end

        % find peak and valley in the distance distribution
        min_prominence = 0;

        for j = 1:length(point_trunk_gap_distance_list)
            min_prominence = min_prominence + mean(point_trunk_gap_distance_list{j});
        end

        min_prominence = mean(min_prominence);
        TF = islocalmax(trunk_branch_distance_list, 'MinProminence', min_prominence);
        TF2 = islocalmin(trunk_branch_distance_list, 'MinProminence', min_prominence);

        split_counter = 0;
        split_clusters_index = {};
        start_index = 1;
        branch_branch_distance_matrix = pdist2(double(pts_list), double(pts_list));

        if sum(TF) == 1 && sum(TF2) == 0
            local_max_index = find(TF);
            peak_index = local_max_index(1);

            split_counter = split_counter + 1;
            split_clusters_index{split_counter} = start_index:peak_index;

            end_index = size(pts_list, 1);

            if end_index - peak_index > 3
                split_counter = split_counter + 1;
                split_clusters_index{split_counter} = peak_index + 1:size(pts_list, 1);
            end

            colors = {'yellow', 'green', 'cyan', 'magenta'};

            if split_counter > 0

                for j = 1:split_counter
                    cluster_counter = cluster_counter + 1;
                    unique_updated_cluster_label(cluster_counter) = cluster_counter;

                    tmp_index = split_clusters_index{j};
                    tmp_pts = pts_list(tmp_index, :);
                    updated_cluster_pts_cell{cluster_counter} = tmp_pts;
                    updated_cluster_pts_list = [updated_cluster_pts_list; tmp_pts];
                    updated_cluster_label = [updated_cluster_label; ones(size(tmp_pts, 1), 1) * cluster_counter];

                    if SHOW_CLUSTER_SPLIT
                        plot3(tmp_pts(:, 1), tmp_pts(:, 2), tmp_pts(:, 3), '.', 'Color', colors{rem(j, length(colors)) + 1}, 'MarkerSize', 30);
                    end

                end

            end

        else
            cluster_counter = cluster_counter + 1;
            unique_updated_cluster_label(cluster_counter) = cluster_counter;
            updated_cluster_pts_cell{cluster_counter} = pts_list;
            updated_cluster_pts_list = [updated_cluster_pts_list; pts_list];
            updated_cluster_label = [updated_cluster_label; ones(size(pts_list, 1), 1) * cluster_counter];

            if SHOW_CLUSTER_SPLIT
                plot3(pts_list(:, 1), pts_list(:, 2), pts_list(:, 3), '.yellow', 'MarkerSize', 30);
            end

        end

        if SHOW_CLUSTER_SPLIT
            title('Branch skeleton points ID')
            xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

            subplot(1, 2, 2)
            plot(branch_id_list, trunk_branch_distance_list); hold on
            plot(branch_id_list, trunk_branch_distance_list, '.r', 'MarkerSize', 20);
            plot(branch_id_list(TF), trunk_branch_distance_list(TF), '*g', 'MarkerSize', 20);
            plot(branch_id_list(TF2), trunk_branch_distance_list(TF2), '*black', 'MarkerSize', 20);
            title('Branch ID vs Distance to trunk')
            xlabel('Branch ID'); ylabel('Distance to trunk'); grid on;
        end

    end

    %%----------------------------------------------------------%%
    %%-------------------Post-Process Cluster-------------------%%
    %% remove over-segmented clusters
    %% 1. find the intersection point of branch cluster and trunk
    %% 2. calculate the closest distance
    %%----------------------------------------------------------%%
    line_distance_threshold = 0.06;
    tmp_unique_updated_cluster_label = [];
    tmp_updated_cluster_pts_cell = {};
    tmp_updated_cluster_pts_list = [];
    tmp_updated_cluster_label = [];
    cluster_counter = 0;
    internode_point_distance = [];

    % precompute internode point-to-point distance statistics
    for i = 1:length(unique_updated_cluster_label)

        cur_cluster_pts = updated_cluster_pts_cell{i};

        [sliced_main_trunk_pts, row, col] = find_internode(double(cur_cluster_pts), refined_main_trunk_pts, 0.1, false);
        internode_point_distance = [internode_point_distance; pdist2(double(sliced_main_trunk_pts(row, :)), double(cur_cluster_pts(col, :)))];

    end

    mean_internode_point_distance = mean(internode_point_distance);
    std_internode_point_distance = std(internode_point_distance);
    point_distance_threshold = mean_internode_point_distance + 2 * std_internode_point_distance;

    for i = 1:length(unique_updated_cluster_label)

        cur_cluster_pts = updated_cluster_pts_cell{i};

        [sliced_main_trunk_pts, row, ~] = find_internode(double(cur_cluster_pts), refined_main_trunk_pts, 0.1, false);

        % fit a 3D line representing the sliced main trunk
        % the main trunk pts used to fit the line shouldn't be too long
        if size(sliced_main_trunk_pts, 1) - row < 5
            tmp_ii = 1:size(sliced_main_trunk_pts, 1);
        else
            tmp_ii = 1:row + 5;
        end

        min_samples = 3; residual_threshold = 0.005; max_trials = 1e3;
        [sliced_vector, ~, ~] = ransac_py(sliced_main_trunk_pts(tmp_ii, :), '3D_Line', min_samples, residual_threshold, max_trials);

        v1 = [cur_cluster_pts(1, :); cur_cluster_pts(2, :)];
        v2 = [cur_cluster_pts(1, :); cur_cluster_pts(3, :)];
        vr = [sliced_vector(1:3); sliced_vector(1:3) + sliced_vector(4:6)];
        [~, nearest_pts1, point_point_distance1, line_line_distance1] = projection_distance(v1, vr);
        [~, nearest_pts2, point_point_distance2, line_line_distance2] = projection_distance(v2, vr);
        [min_line_distance, ~] = min([line_line_distance1, line_line_distance2]);
        [min_point_distance, ~] = min([point_point_distance1, point_point_distance2]);

        if min_line_distance < line_distance_threshold && min_point_distance < point_distance_threshold
            cluster_counter = cluster_counter + 1;
            tmp_unique_updated_cluster_label = [tmp_unique_updated_cluster_label; cluster_counter];
            tmp_updated_cluster_pts_cell{cluster_counter} = cur_cluster_pts;
            tmp_updated_cluster_pts_list = [tmp_updated_cluster_pts_list; cur_cluster_pts];
            tmp_updated_cluster_label = [tmp_updated_cluster_label; ones(size(cur_cluster_pts, 1), 1) * cluster_counter];
        end

        %         figure('Name', ['Cluster ', num2str(i)])
        %         plot3(sliced_main_trunk_pts(:, 1), sliced_main_trunk_pts(:, 2), sliced_main_trunk_pts(:, 3), '.r', 'MarkerSize', 30); hold on
        %         plot3(cur_cluster_pts(:, 1), cur_cluster_pts(:, 2), cur_cluster_pts(:, 3), '.b', 'MarkerSize', 30);
        %         plot3(nearest_pts1(:, 1), nearest_pts1(:, 2), nearest_pts1(:, 3), '.g', 'MarkerSize', 30);
        %         plot3(nearest_pts2(:, 1), nearest_pts2(:, 2), nearest_pts2(:, 3), '.y', 'MarkerSize', 30);
        %         legend('Trunk', 'Cluster', 'NN1', 'NN2');
        %         title(['Point-point distance: ', num2str(min_point_distance, '%.2f')], ...
        %                  ['Line-line distance: ', num2str(min_line_distance, '%.2f')], 'color', [1, 0, 0]);
        %         xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;
    end

    unique_updated_cluster_label = tmp_unique_updated_cluster_label;
    updated_cluster_pts_cell = tmp_updated_cluster_pts_cell;
    updated_cluster_pts_list = tmp_updated_cluster_pts_list;
    updated_cluster_label = tmp_updated_cluster_label;

    %% visualization of initial and refined branch clustering
    figure('Name', '2nd DBSCAN clusters')
    ax1 = subplot(1, 2, 1);
    pcshow(original_pt_normalized, 'markersize', 40); hold on
    plot_dbscan_clusters(crotch_pts, cluster_label);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;
    title(['Initial clusters: ', num2str(length(unique_cluster_label))], 'color', [1, 0, 0]);

    ax2 = subplot(1, 2, 2);
    pcshow(original_pt_normalized, 'markersize', 40); hold on
    plot_dbscan_clusters(updated_cluster_pts_list, updated_cluster_label);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;
    title(['Refined clusters: ', num2str(length(unique_updated_cluster_label))], 'color', [1, 0, 0]);

    Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
    setappdata(gcf, 'StoreTheLink', Link);

    %%---------------------------------------------------------%%
    %%-------------------Branch Segmentation-------------------%%
    %% individual branch segmentation
    %% 1. develop a weighted graph for each cluster
    %%    - exclude other cluster points in the graph
    %% 2. find the longest MST in each graph as entire branch
    %% notice: # of clusters may change because
    %%         - a branch could be separated into several clusters
    %%         - multi branches could be grouped into one cluster
    %%---------------------------------------------------------%%
    disp('===================Branch Segmentation===================');
    % build weighted graph which excludes main trunk pts and refined branch pts
    branch_distance_th = load_parameters(paras, 'branch_distance_th_lambda2', 0.02);
    rest_pts_adj = MST_adj_matrix(rest_pts_idx, rest_pts_idx);
    rest_pts_density = P.spls_density(rest_pts_idx);

    branch_mode = load_parameters(paras, 'subgraph_refine_mode', 'distance');
    [adj_matrix, adj_idx, density_weight, inv_density_weight, distance_weight] = refine_adj_matrix(rest_pts, rest_pts_adj, rest_pts_density, branch_distance_th, branch_mode);
    density_weight_normalized = normalize(density_weight, 'range');
    distance_weight_normalized = normalize(distance_weight, 'range');
    coefficient_density_weight = load_parameters(paras, 'subgraph_edge_coefficient_alpha2', 0.8);

    disp(['distance threshold lambda2: ' num2str(branch_distance_th)]);
    disp(['subgraph refine mode: ' branch_mode]);
    disp(['subgraph edge coefficient alpha2: ' num2str(coefficient_density_weight)]);

    branch_counter = 0;
    branch_pts_idx = {}; % index in terms of P.spls
    valid_cluster_pts_cell = {};
    visited = zeros(size(P.spls, 1), 1); % if points have been already assigned to one of primary branches
    visited_id = cell(size(P.spls, 1), 1);
    MST_list = {};
    MST_node_list = {};

    [~, updated_cluster_pts_index_in_rest_pts] = ismember(updated_cluster_pts_list, rest_pts, 'row');

    for i = 1:length(unique_updated_cluster_label)

        cur_cluster_pts = updated_cluster_pts_cell{i};
        [~, cur_cluster_pts_index_in_spls] = ismember(cur_cluster_pts, P.spls, 'row');
        [~, cur_cluster_pts_idx] = ismember(cur_cluster_pts, rest_pts, 'row');
        rest_cluster_pts_idx = setdiff(updated_cluster_pts_index_in_rest_pts, cur_cluster_pts_idx);

        % remove other cluster points from the graph
        adj_idx_trans = adj_idx';
        [~, tmp_index] = ismember(rest_cluster_pts_idx, adj_idx_trans);
        rest_index = setdiff(1:size(adj_idx, 2), tmp_index);
        adj_idx_rest = adj_idx(:, rest_index);
        density_weight_normalized_rest = density_weight_normalized(rest_index);
        distance_weight_normalized_rest = distance_weight_normalized(rest_index);
        rest_MST_weight = coefficient_density_weight * density_weight_normalized_rest + (1 - coefficient_density_weight) * distance_weight_normalized_rest;
        rest_weighted_graph = graph(adj_idx_rest(1, :), adj_idx_rest(2, :), rest_MST_weight);

        MSTs_length = zeros(length(cur_cluster_pts_idx), 1);

        % go over each point in the cluster
        for j = 1:length(cur_cluster_pts_idx)
            cur_pts_in_MST_idx = cur_cluster_pts_idx(j); % index in rest_pts
            [MST, ~] = minspantree(rest_weighted_graph, 'Type', 'tree', 'Root', findnode(rest_weighted_graph, cur_pts_in_MST_idx));
            MST_length = size(MST.Edges, 1);
            MSTs_length(j) = MST_length;
        end

        [MST_max_length, MST_max_idx] = max(MSTs_length);

        % points are not connected in MST
        if MST_max_length <= 3
            disp(['===================SKIP BRNACH ' num2str(i) 'Due to MST <= 3 ', num2str(MST_max_length),  ' ==================='])
            continue
        end

        branch_counter = branch_counter + 1;
        valid_cluster_pts_cell{branch_counter} = cur_cluster_pts;

        [~, MST_max_idx] = max(MSTs_length);
        branch_point_idx_max_MST = cur_cluster_pts_idx(MST_max_idx); % index in rest_pts

        [MST, ~] = minspantree(rest_weighted_graph, 'Type', 'tree', 'Root', findnode(rest_weighted_graph, branch_point_idx_max_MST));
        MST_nodes = table2array(MST.Edges);
        MST_nodes_unique = unique(MST_nodes(:, 1:2));
        MST_list{branch_counter} = MST;
        MST_node_list{branch_counter} = MST_nodes_unique;

        [~, tmp] = ismember(rest_pts(MST_nodes_unique, :), P.spls, 'row'); % the entire branch points (index in spls)
        tmp = unique([tmp; cur_cluster_pts_index_in_spls]); %make sure cluster points are included
        visited(tmp) = visited(tmp) + 1;

        for k = 1:length(tmp)
            visited_id{tmp(k)} = [visited_id{tmp(k)}, branch_counter];
        end

        branch_pts_idx{branch_counter} = tmp;

    end

    if SAVE_FIG
        filename = fullfile(output_folder, 'weighted_graph_wi_trunk');
        saveas(gcf, filename);
        axis off;
        print('-painters', '-dpdf', '-fillpage', '-r300', filename);
    end

    figure('Name', 'Branch count visualization')
    ax1 = subplot(1, 2, 1);
    pcshow(original_pt_normalized, 'MarkerSize', 30); hold on
    plot_by_weight(P.spls, visited);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    %%---------------------------------------------------------%%
    %%-------------------Post-Process Branch-------------------%%
    %% merge over-segmented branch to existing branches
    %% 1. find over-segmented points and their root ID
    %% 2. specify 'k' in spectral clustering based on #root ID
    %% 3. compute the center of each cluster
    %% 4. merge to the closest existing cluster (Discarded)
    %% 5. merge by matching growing direction!
    %%---------------------------------------------------------%%
    overcount_branch_pts_index = find(visited > 1);
    overcount_branch_pts_cluster = visited_id(overcount_branch_pts_index);
    overcount_branch_pts_group = unique(cell2mat(overcount_branch_pts_cluster), 'rows');
    overcount_branch_pts = P.spls(overcount_branch_pts_index, :);
    [~, overcount_branch_pts_index2] = ismember(overcount_branch_pts, rest_pts, 'row');
    overcount_adj_matrix = adj_matrix(overcount_branch_pts_index2, overcount_branch_pts_index2);

    ax2 = subplot(1, 2, 2);
    pcshow(original_pt_normalized, 'MarkerSize', 30); hold on
    scatter3(overcount_branch_pts(:, 1), overcount_branch_pts(:, 2), overcount_branch_pts(:, 3), sizep, '.');
    plot_connectivity(overcount_branch_pts, overcount_adj_matrix, sizee, colore);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
    setappdata(gcf, 'StoreTheLink', Link);

    for j = 1:branch_counter
        cur_branch_pts_index = branch_pts_idx{j};
        intersect_index = intersect(cur_branch_pts_index, overcount_branch_pts_index);
        branch_pts_idx{j} = setdiff(branch_pts_idx{j}, intersect_index);
    end

    figure('Name', 'Entire branch identification')
    pcshow(original_pt_normalized, 'MarkerSize', 30); hold on

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};

    for j = 1:branch_counter
        cur_branch_pts_idx = branch_pts_idx{j};
        cur_branch_pts = P.spls(cur_branch_pts_idx, :);
        plot3(cur_branch_pts(:, 1), cur_branch_pts(:, 2), cur_branch_pts(:, 3), '.', 'Color', colors{rem(j, length(colors)) + 1}, 'MarkerSize', 20);
    end

    title(['Clusters: ', num2str(branch_counter)], 'color', [1, 0, 0]);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    % merge
    noise_threshold = 10;
    merge_threshold = 0.2;

    disp(['merge #point threshold: ' num2str(noise_threshold)]);
    disp(['merge distance threshold: ' num2str(merge_threshold)]);

    for m = 1:size(overcount_branch_pts_group, 1)
        visited_group = overcount_branch_pts_group(m, :);
        tmp_index = cellfun(@(x)all(x == visited_group), overcount_branch_pts_cluster);
        tmp_pts_index = overcount_branch_pts_index(tmp_index);
        overcount_branch_pts = P.spls(tmp_pts_index, :);

        % discard small clusters
        if size(overcount_branch_pts, 1) > noise_threshold

            num_split_cluster = length(visited_group);
            overcount_branch_cluster_label = spectralcluster(overcount_branch_pts, num_split_cluster);
            unique_overcount_branch_cluster_label = unique(overcount_branch_cluster_label);

            assert(length(unique_overcount_branch_cluster_label)==length(visited_group), 'Error');

            visited2 = zeros(num_split_cluster, 1);
            for j = 1:length(unique_overcount_branch_cluster_label)

                tmp_cluster_label = unique_overcount_branch_cluster_label(j);
                tmp_index = overcount_branch_cluster_label == tmp_cluster_label;
                overcount_branch_cluster_pts = overcount_branch_pts(tmp_index, :); 

                angle_diff_list = [];
                for k = 1:length(visited_group)
                    group_id = visited_group(k);
                    neighbor_cluster_pts = valid_cluster_pts_cell{group_id};

                    % find the closest points between two clusters
                    distance_matrix = pdist2(double(neighbor_cluster_pts), double(overcount_branch_cluster_pts));
                    [d_min, d_tmp_index] = min(distance_matrix(:));
                    [row, col] = ind2sub(size(distance_matrix), d_tmp_index);
                    tmp_pts1 = neighbor_cluster_pts(row-1, :);
                    tmp_pts2 = neighbor_cluster_pts(row, :);
                    tmp_pts3 = overcount_branch_cluster_pts(col, :);
                    critical_pts = [tmp_pts1; tmp_pts2; tmp_pts3];

                    % compute angle
                    [pts_vector2, pts_vector_angle2] = point_to_point_angle(critical_pts);
                    angle_diff_list = [angle_diff_list; pts_vector_angle2];
                end

                [~, tmp_index] = mink(angle_diff_list, length(visited_group));
                while visited2(tmp_index(1))
                    tmp_index(1) = [];
                end
                visited2(tmp_index(1)) = 1;
                [~, ii] = ismember(overcount_branch_cluster_pts, P.spls, 'row');
                merge_group_id = visited_group(tmp_index(1));
                branch_pts_idx{merge_group_id} = [branch_pts_idx{merge_group_id}; ii];

            end

        end

    end

    figure('Name', 'Refined entire branch identification')
    pcshow(original_pt_normalized, 'MarkerSize', 30); hold on

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta', 'white'};

    for i = 1:branch_counter
        cur_branch_pts_idx = branch_pts_idx{i};
        cur_branch_pts = P.spls(cur_branch_pts_idx, :);
        tmp_pts = cur_branch_pts(1, :);
        plot3(cur_branch_pts(:, 1), cur_branch_pts(:, 2), cur_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 20);
        text(tmp_pts(1), tmp_pts(2), tmp_pts(3) + 0.02, num2str(i), 'Color', 'red', 'HorizontalAlignment', 'left', 'FontSize', 12);
    end

    title(['Clusters: ', num2str(branch_counter)], 'color', [1, 0, 0]);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    %%-------------------------------------------------------------------%%
    %%-------------------Primary Branch Identification-------------------%%
    %% same to main trunk detection
    %% find the point providing the maximum path weight
    %%-------------------------------------------------------------------%%
    primary_branch_pts_idx = {}; % index in terms of P.spls

    for j = 1:branch_counter
        cur_branch_pts_idx = branch_pts_idx{j};
        branch_pts = P.spls(cur_branch_pts_idx, :);
        MST = MST_list{j};
        MST_node = MST_node_list{j};

        [~, ~, col] = find_internode(branch_pts, refined_main_trunk_pts, 0.1, false);
        [~, branch_internode_index_in_rest_pts] = ismember(branch_pts(col, :), rest_pts, 'row');

        % internode might not be included in MST (small possibility)
        d_m = pdist2(double(branch_pts), double(branch_pts(col, :)));
        [~, tmp_index] = mink(d_m, size(branch_pts, 1));

        while ~ismember(branch_internode_index_in_rest_pts, MST_node)
            tmp_index(1) = [];
            col = tmp_index(1);
            [~, branch_internode_index_in_rest_pts] = ismember(branch_pts(col, :), rest_pts, 'row');
        end

        shortest_path_nodes = cell(size(branch_pts, 1), 1);
        shortest_path_distance = zeros(size(branch_pts, 1), 1);

        for k = 1:size(branch_pts, 1)
            cur_branch_pts = branch_pts(k, :);
            [~, tmp] = ismember(cur_branch_pts, rest_pts, 'row'); % index in terms of rest_pts (which MST built upon)
            [node, distance] = shortestpath(MST, branch_internode_index_in_rest_pts, tmp);

            if isempty(node)
                distance = 0;
            end

            shortest_path_nodes{k} = node;
            shortest_path_distance(k) = distance;
        end

        [~, max_weight_idx] = max(shortest_path_distance);
        shortest_path_node = shortest_path_nodes{max_weight_idx}; % The node was already sorted based on its correponding distance to the source node
        [~, tmp] = ismember(rest_pts(shortest_path_node, :), P.spls, 'row');
        primary_branch_pts_idx{j} = tmp;
    end

    figure('Name', 'Primary branch identification')
    ax1 = subplot(1, 2, 1);
    pcshow(original_pt_normalized, 'MarkerSize', 30); hold on

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta', 'white'};

    for i = 1:branch_counter
        cur_branch_pts_idx = primary_branch_pts_idx{i};
        cur_branch_pts = P.spls(cur_branch_pts_idx, :);
        tmp_pts = cur_branch_pts(1, :);
        plot3(cur_branch_pts(:, 1), cur_branch_pts(:, 2), cur_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 20);
        text(tmp_pts(1), tmp_pts(2), tmp_pts(3) + 0.02, num2str(i), 'Color', 'red', 'HorizontalAlignment', 'left', 'FontSize', 12);
    end

    title(['Clusters: ', num2str(branch_counter)], 'color', [1, 0, 0]);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    ax2 = subplot(1, 2, 2);
    pcshow(original_pt_normalized, 'MarkerSize', 30); hold on
    %% visualization of field measurement
    for i = 1:length(files)
        filename = files(i).name;
        filepath = files(i).folder;
        fieldname = split(filename, '.');
        tmp_pc = pcread([filepath '\' filename]);
        desired_pt = 5000;

        if desired_pt <= tmp_pc.Count
            ratio = desired_pt / tmp_pc.Count;
            tmp_pc = pcdownsample(tmp_pc, 'random', ratio);
        end

        tmp_pc_location = tmp_pc.Location;

        fieldname = fieldname{1};

        if contains(fieldname, 'Section')
            color = split(fieldname, '_');
            color = color{2};

            if strcmp(color, 'Orange')
                color = 'Yellow';
            elseif strcmp(color, 'Black')
                color = 'White';
            end

            plot3(tmp_pc_location(:, 1), tmp_pc_location(:, 2), tmp_pc_location(:, 3), '.', 'Color', color, 'markersize', 20)
        end

    end

    title('Field measurement visualization', 'color', [1, 0, 0]);
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
    setappdata(gcf, 'StoreTheLink', Link);

    if BRANCH_REFINEMENT
        %%-----------------------------------------------------%%
        %%-------------------Branch CPC-------------------%%
        %%-----------------------------------------------------%%
        branch_refinement_options.sphere_radius = 0.01;
        branch_refinement_options.maximum_length = 0.002;

        primary_branch_pc_list = [];
        primary_center = [];
        primary_radius = [];
        primary_confidence = [];

        side_branch_pc_list = [];
        side_center = [];
        side_radius = [];
        side_confidence = [];

        internode_list = [];
        primary_spline_curve_list = [];
        primary_spline_pts = [];

        primary_center_size = [];
        primary_spline_size = [];
        side_center_size = [];

        %% build kdtree and retrieve branch surface points
        kdtree = KDTreeSearcher(original_pt_normalized_location);

        for i = 1:branch_counter
            cur_branch_pts_idx = branch_pts_idx{i};
            cur_primary_branch_pts_idx = primary_branch_pts_idx{i};
            cur_side_branch_pts_idx = setdiff(cur_branch_pts_idx, cur_primary_branch_pts_idx);

            % CAUTION!! radius might have NaN, which is intentional
            CPC = branch_refinement(P, kdtree, cur_primary_branch_pts_idx, branch_refinement_options);

            primary_branch_cpc_optimized_center = CPC.cpc_optimzed_center_median;
            primary_branch_cpc_optimized_radius = CPC.cpc_optimized_radius_median;
            primary_branch_cpc_optimized_confidence = CPC.cpc_optimized_confidence_median;
            primary_branch_pc = CPC.branch_pc;

            primary_branch_pc_list = [primary_branch_pc_list; primary_branch_pc];
            primary_center = [primary_center; primary_branch_cpc_optimized_center];
            primary_radius = [primary_radius; primary_branch_cpc_optimized_radius];
            primary_confidence = [primary_confidence; primary_branch_cpc_optimized_confidence];
            primary_center_size = [primary_center_size; size(primary_branch_cpc_optimized_center, 1)];

            % find trunk/branch internode
            [sliced_main_trunk_pts, row, col] = find_internode(primary_branch_cpc_optimized_center, P.trunk_cpc_optimized_center, 0.2);
            trunk_internode = sliced_main_trunk_pts(row, :);
            branch_internode = primary_branch_cpc_optimized_center(col, :);
            internode_list = [internode_list; [trunk_internode, branch_internode]];

            %% fit a spline and uniformly sample points from the spline
            primary_branch_pts_and_trunk_node = [trunk_internode; primary_branch_cpc_optimized_center];
            primary_branch_pts_and_trunk_node = double(primary_branch_pts_and_trunk_node);

            M = size(primary_branch_cpc_optimized_center, 1) * 2; % #sample

            if M < 8
                M = 10;
            end

            [curve, uniform_xyz] = spline_interpolation(primary_branch_pts_and_trunk_node, M);
            primary_spline_curve_list = [primary_spline_curve_list; curve];
            primary_spline_pts = [primary_spline_pts; uniform_xyz];
            primary_spline_size = [primary_spline_size; size(uniform_xyz, 1)];

            if ~isempty(cur_side_branch_pts_idx)
                CPC = branch_refinement(P, kdtree, cur_side_branch_pts_idx, branch_refinement_options);

                side_branch_cpc_optimized_center = CPC.cpc_optimzed_center_median;
                side_branch_cpc_optimized_radius = CPC.cpc_optimized_radius_median;
                side_branch_cpc_optimized_confidence = CPC.cpc_optimized_confidence_median;
                side_branch_pc = CPC.branch_pc;

                side_branch_pc_list = [side_branch_pc_list; side_branch_pc];
                side_center = [side_center; side_branch_cpc_optimized_center];
                side_radius = [side_radius; side_branch_cpc_optimized_radius];
                side_confidence = [side_confidence; side_branch_cpc_optimized_confidence];
                side_center_size = [side_center_size; size(side_branch_cpc_optimized_center, 1)];
            end

        end

        P.primary_branch_pc = pccat(primary_branch_pc_list);
        P.primary_branch_center = primary_center;
        P.primary_branch_radius = primary_radius;
        P.primary_branch_confidence = primary_confidence;
        P.primary_center_size = primary_center_size;

        P.primary_spline_curve = primary_spline_curve_list;
        P.primary_spline_center = primary_spline_pts;
        P.primary_spline_size = primary_spline_size;
        P.internode_list = internode_list;

        P.side_branch_pc = pccat(side_branch_pc_list);
        P.side_branch_center = side_center;
        P.side_branch_radius = side_radius;
        P.side_branch_confidence = side_confidence;
        P.side_center_size = side_center_size;

        save(new_skel_filepath, 'P');

        disp(['sphere radius: ' num2str(branch_refinement_options.sphere_radius)]);
        disp(['maximum length: ' num2str(branch_refinement_options.maximum_length)]);
        disp(['M: ' num2str(M)]);
        disp('===================Finish Branch Refinement by CPC and Saved===================');

        invalid_primary_branch_index = isnan(P.primary_branch_radius);
        invalid_side_branch_index = isnan(P.side_branch_radius);
        valid_centers = [P.primary_branch_center(~invalid_primary_branch_index, :); P.side_branch_center(~invalid_side_branch_index, :)];
        valid_radii = [P.primary_branch_radius(~invalid_primary_branch_index); P.side_branch_radius(~invalid_side_branch_index)];

        invalid_centers = [P.primary_branch_center(invalid_primary_branch_index, :); P.side_branch_center(invalid_side_branch_index, :)];
        invalid_radii = [P.primary_branch_radius(invalid_primary_branch_index); P.side_branch_radius(invalid_side_branch_index)];

        figure('Name', 'Optimized branch skeleton pts')
        ax1 = subplot(1, 2, 1);
        pcshow(P.branch_pc, 'Markersize', 30); hold on
        plot3(P.primary_branch_center(:, 1), P.primary_branch_center(:, 2), P.primary_branch_center(:, 3), '.r', 'Markersize', 30)
        plot3(P.side_branch_center(:, 1), P.side_branch_center(:, 2), P.side_branch_center(:, 3), '.b', 'Markersize', 30)
        plot3(invalid_centers(:, 1), invalid_centers(:, 2), invalid_centers(:, 3), '.g', 'Markersize', 30)
        xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); axis equal; grid on;

        ax2 = subplot(1, 2, 2);
        pcshow(P.branch_pc, 'Markersize', 30); hold on
        plot_by_weight(valid_centers, valid_radii / trunk_radius);
        xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); axis equal; grid on;
    
        Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
        setappdata(gcf, 'StoreTheLink', Link);
    end

    disp('================Segmention Done================');

    if LOGGING
        diary off
        movefile('logfile', log_filepath);
    end

    clc; clear; close all;
end
