function trait(tree_id, skel_folder, exp_id, excel_filename, options)

    SHOW = options.SHOW;
    SHOW_BRANCH = options.SHOW_BRANCH;
    SAVE = options.SAVE;
    CLEAR = options.CLEAR;
    TO_FUSION = options.TO_FUSION;

    skel_filename_format = '_contract_*_skeleton.mat';
    new_skel_folder = fullfile(skel_folder, '..', 'segmentation');
    skel_filename = search_skeleton_file(tree_id, fullfile(new_skel_folder, exp_id), skel_filename_format);
    output_folder = fullfile(skel_folder, '..', 'characterization', exp_id);
    cad_save_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\Fusion\';

    %% create folder to save results
    if ~exist(output_folder, 'dir')
        mkdir(output_folder)
    end

    if ~exist(cad_save_folder, 'dir')
        mkdir(cad_save_folder)
    end

    skel_filepath = fullfile(new_skel_folder, exp_id, skel_filename);
    load(skel_filepath, 'P'); % P results from skeleton operation

    start = 0;
    spline_start = 0;
    T = table();

    branch_counter = length(P.primary_center_size);

    if SHOW
        branch_number_fig = figure(100);
        set(branch_number_fig, 'Name', 'Branch Mapping', 'Color', 'white');
        title([num2str(branch_counter), ' branches']);
        pcshow(P.original_pt, 'markersize', 30)
        set(gcf, 'color', 'white'); set(gca, 'color', 'white')
        hold on
    end

    trunk_skeleton_pts = P.trunk_cpc_optimized_center;
    trunk_internode_ratio = P.trunk_internode_distance_ratio;

    branch_internode_ratio = [];
    branch_angle_list = [];
    branch_diameter_list = [];
    branch_pts_cell = {};

    for i = 1:branch_counter
        primary_branch_pts_size = P.primary_center_size(i);
        index = start + 1:start + primary_branch_pts_size;
        primary_branch_pts = P.primary_branch_center(index, :);
        primary_branch_pts_radius = P.primary_branch_radius(index);
        primary_branch_pts_radius(primary_branch_pts_radius > P.trunk_radius / 2) = NaN;

        % find the internode
        [sliced_main_trunk_pts, row, col] = find_internode(double(primary_branch_pts), double(trunk_skeleton_pts), 0.2);
        trunk_internode = sliced_main_trunk_pts(row, :);
        branch_internode = primary_branch_pts(col, :);

        [~, index] = ismember(trunk_internode, trunk_skeleton_pts, 'row');
        ratio_distance = trunk_internode_ratio(index);
        trunk_radius = P.trunk_cpc_optimized_radius(index);

        %% use the spline information to estimate diameter and angle
        curve = P.primary_spline_curve(i);
        primary_spline_pts_size = P.primary_spline_size(i);
        index = spline_start + 1:spline_start + primary_spline_pts_size;
        primary_spline_pts = P.primary_spline_center(index, :);
        kdtree = KDTreeSearcher(primary_branch_pts);
        [spline_nn_index, ~] = knnsearch(kdtree, primary_spline_pts, 'K', 3);
        primary_spline_pts_radius = median(primary_branch_pts_radius(spline_nn_index), 2, 'omitnan');

        % fit trunk vector - only use trunk points that are close to the
        % internode
        min_samples = 3; residual_threshold = 0.005; max_trials = 1e3;
        [v1, inliers, ~] = ransac_py(sliced_main_trunk_pts, '3D_Line', min_samples, residual_threshold, max_trials);

        if v1(end) < 0
            v1(end) = -v1(end);
        end

        sliced_main_trunk_pts_inlier = sliced_main_trunk_pts(inliers == 1, :);
        sliced_main_trunk_pts_outlier = sliced_main_trunk_pts(inliers ~= 1, :);

        N = 4;
        [segment_vectors, vertical_angle, N] = find_branch_angle(trunk_internode, trunk_radius, v1, primary_spline_pts, N);

        % radius
        M = 4; % #skeleton points used in radius
        tmp_radius = primary_spline_pts_radius;

        if length(tmp_radius) < M
            index = 1:length(tmp_radius);
        else
            index = 1:M;
        end

        radius = median(tmp_radius(index), 'omitnan') * 1e3;

        if TO_FUSION
            branch_internode_ratio = [branch_internode_ratio, ratio_distance];
            branch_angle_list = [branch_angle_list, vertical_angle];
            branch_diameter_list = [branch_diameter_list, radius];
            branch_pts_cell{i} = primary_branch_pts;
        end

        if SHOW_BRANCH
            figure('Name', ['Branch ', num2str(i), ' spline'])
            subplot(1, 2, 1)
            plot3(sliced_main_trunk_pts_inlier(:, 1), sliced_main_trunk_pts_inlier(:, 2), sliced_main_trunk_pts_inlier(:, 3), '.r', 'Markersize', 30); hold on
            plot3(sliced_main_trunk_pts_outlier(:, 1), sliced_main_trunk_pts_outlier(:, 2), sliced_main_trunk_pts_outlier(:, 3), '.y', 'Markersize', 30);
            quiver3(v1(1), v1(2), v1(3), v1(4), v1(5), v1(6));
            plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.g', 'Markersize', 30);
            fnplt(curve, 'b', 2);
            legend('Inlier', 'Outlier', 'Trunk vector', 'Entire branch', 'Spline')
            xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

            subplot(1, 2, 2)
            plot3(sliced_main_trunk_pts_inlier(:, 1), sliced_main_trunk_pts_inlier(:, 2), sliced_main_trunk_pts_inlier(:, 3), '.r', 'Markersize', 30); hold on
            plot3(sliced_main_trunk_pts_outlier(:, 1), sliced_main_trunk_pts_outlier(:, 2), sliced_main_trunk_pts_outlier(:, 3), '.y', 'Markersize', 30);
            quiver3(v1(1), v1(2), v1(3), v1(4), v1(5), v1(6));
            plot3(primary_spline_pts(:, 1), primary_spline_pts(:, 2), primary_spline_pts(:, 3), '.g', 'Markersize', 30);
            quiver3(segment_vectors(1:N, 1), segment_vectors(1:N, 2), segment_vectors(1:N, 3), ...
                segment_vectors(1:N, 4), segment_vectors(1:N, 5), segment_vectors(1:N, 6), ...
                1, 'LineWidth', 2);
            legend('Inlier', 'Outlier', 'Trunk vector', 'Branch vector', 'Entire branch')
            title('Vertical angle: ', num2str(vertical_angle, '%.2f'))
            xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

            figure('Name', ['Branch ', num2str(i), ' radius'])
            subplot(1, 2, 1)
            plot_by_weight(primary_branch_pts, primary_branch_pts_radius / P.trunk_radius)
            title('Branch skeleton points radius heatmap')
            xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

            subplot(1, 2, 2)
            histogram(primary_branch_pts_radius)
            title('Branch skeleton points radius hitogram')
        end

        if SHOW
            text_loc = 1;
            figure(100)
            plot3(primary_spline_pts(:, 1), primary_spline_pts(:, 2), primary_spline_pts(:, 3), '.r', 'Markersize', 30); hold on
            plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.b', 'Markersize', 20);
            text(primary_branch_pts(text_loc, 1), primary_branch_pts(text_loc, 2), primary_branch_pts(text_loc, 3) + 0.03, num2str(i), 'Color', 'red', 'HorizontalAlignment', 'left', 'FontSize', 12);
        end

        if SAVE
            T = [T; {tree_id, num2str(i), num2str(vertical_angle, '%.2f'), num2str(radius, '%.2f')}];
        end

        start = start + primary_branch_pts_size;
        spline_start = spline_start + primary_spline_pts_size;
    end

    if TO_FUSION
        P.branch_internode_ratio = branch_internode_ratio;
        P.branch_diameter = branch_diameter_list;
        P.branch_angle = branch_angle_list;
        P.branch_pts_list = branch_pts_cell;
        save(fullfile(cad_save_folder, skel_filename), 'P');
    end

    if SAVE
        T.Properties.VariableNames = {'Filename', 'Branch ID', 'Vertical_Croth_Angle-Degree', 'Primary_Branch_Radius-mm'};
        branch_excel_filepath = fullfile(output_folder, [tree_id excel_filename]);
        writetable(T, branch_excel_filepath, 'Sheet', 'Branch_Level_Traits_1')
    end

    if CLEAR
        clc; clear; close all;
    end

end
