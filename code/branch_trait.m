function T = branch_trait(skel_folder, output_folder, tree_id, matched_qsm_table, options)

    SHOW = options.SHOW;
    SHOW_BRANCH = options.SHOW_BRANCH;
    SAVE = options.SAVE;
    OUTPUT = options.OUTPUT;

    % output for CAD tree modeling
    cad_save_folder = fullfile(output_folder, 'Fusion');
    if ~exist(cad_save_folder, 'dir') && OUTPUT
        mkdir(cad_save_folder)
    end

    skel_filename_format = '_contract_*_skeleton.mat';
    skel_filename = search_skeleton_file(tree_id, skel_folder, skel_filename_format);
    if isnan(skel_filename)
        disp('===================Characterization Failure===================');
        error([skel_filename 'Not Found']);
    end
    % load skeleton and segmentation information
    skel_filepath = fullfile(skel_folder, skel_filename);
    load(skel_filepath, 'P'); % P results from skeleton operation

    start = 0;
    spline_start = 0;
    cut_branch_counter = 0;
    T = table();
    branch_counter = length(P.primary_center_size);

    %% tree visualization
    if SHOW
        branch_number_fig = figure(100);
        set(branch_number_fig, 'Name', 'Branch Mapping', 'Color', 'white');
        % calculate y offset (1 m)
        y_offset = (str2double(tree_id(5:end))-1) * 1;
        % offset y to align with orchard setup
        num_point = size(P.original_pt.Location, 1);
        pc_color = uint8(repmat([0, 0, 0], num_point, 1));
        offset_location = P.original_pt.Location;
        offset_location(:, 2) = offset_location(:, 2) + y_offset;
        offset_pc = pointCloud(offset_location, 'Color', pc_color);
        % find top point to text
        [~, top_point_index] = max(offset_location(:, 3));
        top_point = offset_location(top_point_index, :);
        % downsample for lighter visualization
        desired_pt = 30000;
        if offset_pc.Count > desired_pt
            ratio = desired_pt / offset_pc.Count;
            offset_pc = pcdownsample(offset_pc, 'random', ratio); % visualization purpose only!
        end
        % plot
        pcshow(offset_pc, 'markersize', 30)
        set(gcf, 'color', 'white'); set(gca, 'color', 'white')
        hold on
        text(top_point(1), top_point(2), 2.5, tree_id, 'Color', 'Black', 'HorizontalAlignment', 'center', 'FontSize', 15);
    end

    %% characterization
    trunk_skeleton_pts = P.trunk_cpc_optimized_center;
    trunk_skeleton_pts_root_z = min(trunk_skeleton_pts(:, 3));
    trunk_internode_ratio = P.trunk_internode_distance_ratio;

    branch_internode_ratio = [];
    branch_angle_list = [];
    branch_diameter_list = [];
    branch_pts_cell = {};
    branch_pts_radius_cell = {};

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
        branch_internode_height = branch_internode(3) - trunk_skeleton_pts_root_z;

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
        primary_spline_pts_radius = fillmissing(primary_spline_pts_radius, 'linear');
%         primary_spline_pts_radius = primary_branch_pts_radius(~isnan(primary_branch_pts_radius));
        primary_branch_pts_distance = sqrt(sum(diff([trunk_internode; primary_spline_pts]).^2, 2));        % append trunk internode to the first
        primary_branch_length = cumsum(primary_branch_pts_distance);

        if all(isnan(primary_branch_pts_radius)) || isempty(primary_spline_pts_radius)
            disp(['===================SKIP BRNACH ' num2str(i) 'Due to All NaN ==================='])
            start = start + primary_branch_pts_size;
            spline_start = spline_start + primary_spline_pts_size;
            continue
        end

        assert(~any(isnan(primary_spline_pts_radius)), 'Found NaN')

        % fit trunk vector - only use trunk points that are close to the
        % internode
        min_samples = 3; residual_threshold = 0.005; max_trials = 1e3;
        [v1, inliers, ~] = ransac_py(sliced_main_trunk_pts, '3D_Line', min_samples, residual_threshold, max_trials);

        if v1(end) < 0
            v1(end) = -v1(end);
        end

        sliced_main_trunk_pts_inlier = sliced_main_trunk_pts(inliers == 1, :);
        sliced_main_trunk_pts_outlier = sliced_main_trunk_pts(inliers ~= 1, :);

        % angle
        N = options.CHAR_PARA.angle_K;
        [segment_vectors, vertical_angle, N] = find_branch_angle(trunk_internode, trunk_radius, v1, primary_spline_pts, N);

        % radius
        s = options.CHAR_PARA.diameter_start_idx;
        M = options.CHAR_PARA.diameter_K; % #skeleton points used in radius
        tmp_radius = primary_spline_pts_radius;
        if length(tmp_radius) < M+s
            r_index = 1:length(tmp_radius);
        else
            r_index = s:M+s;
        end
        radius = median(tmp_radius(r_index), 'omitnan') * 1e3;

        % retrieve manual measurements
        % find the row indices where the specified values occur in the respective columns
        if ~isempty(matched_qsm_table)
            [row_index, ~] = find(matched_qsm_table.BranchID_df2 == i & strcmp(matched_qsm_table.Filename, tree_id));
            selected_row = matched_qsm_table(row_index, :);
            gt_branch_diameter = selected_row.Manual_Branch_Diameter_mm;
        else
            gt_branch_diameter = nan;
        end

        % assign colors to points
        branch_color = zeros(length(index), 3); % Initialize with default color

        % radius unit - mm, lenght unit - m
        if (radius > 10) && (cut_branch_counter < 2)
            cut_branch_counter = cut_branch_counter + 1;
            % find the cutting point index
            cutting_point_index = find(primary_branch_length > 0.1, 1);
            %% check if it is FP or TP
%             if (isnan(gt_branch_diameter)) || (gt_branch_diameter > 20)
%                 status_color = [1, 0, 0];             % TP FP [1, 0.6, 0]
%             else
%                 status_color = [1, 0.6, 0]; 
%             end
            status_color = [1, 0, 0];
            branch_color(cutting_point_index+1:end, :) = repmat(status_color, length(index) - cutting_point_index, 1); % Red color for points after cutting point
        elseif primary_branch_length(end) > 0.4
            % find the cutting point index
            cutting_point_index = find(primary_branch_length > 0.4, 1);
            status_color = [0, 0, 1];
            branch_color(cutting_point_index+1:end, :) = repmat(status_color, length(index) - cutting_point_index, 1); % Red color for points after cutting point
        else
            cutting_point_index = length(index);
        end
        % set the color of rest of points to green
        branch_color(1:cutting_point_index, :) = repmat([0, 1, 0], cutting_point_index, 1); % Default color (blue) for points before cutting point

        if OUTPUT
            branch_internode_ratio = [branch_internode_ratio, ratio_distance];
            branch_angle_list = [branch_angle_list, vertical_angle];
            branch_diameter_list = [branch_diameter_list, radius];
            branch_pts_cell{end+1} = primary_spline_pts;
            branch_pts_radius_cell{end+1} = primary_spline_pts_radius;
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
            scatter3(primary_spline_pts(:, 1), primary_spline_pts(:, 2) + y_offset, primary_spline_pts(:, 3),  50, branch_color, 'filled'); hold on
            text(primary_spline_pts(text_loc, 1)+0.05, primary_spline_pts(text_loc, 2)+y_offset+0.05, primary_spline_pts(text_loc, 3)+0.05, num2str(i), 'Color', 'red', 'HorizontalAlignment', 'left', 'FontSize', 12);
            % plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.b', 'Markersize', 20);
        end

        if SAVE
            T = [T; {tree_id, num2str(i), num2str(vertical_angle, '%.2f'), num2str(radius*2, '%.2f'), num2str(branch_internode_height*100, '%.2f'), num2str(primary_branch_length(end)*100, '%.2f')}];
        end

        start = start + primary_branch_pts_size;
        spline_start = spline_start + primary_spline_pts_size;
    end

    if OUTPUT
        P.branch_internode_ratio = branch_internode_ratio;
        P.branch_diameter = branch_diameter_list;
        P.branch_angle = branch_angle_list;
        P.branch_pts_list = branch_pts_cell;
        P.branch_pts_radius_list = branch_pts_radius_cell;
        save(fullfile(cad_save_folder, skel_filename), 'P');
    end

    if SAVE
        T.Properties.VariableNames = {'Filename', 'Branch ID', 'Vertical_Croth_Angle-Degree', 'Primary_Branch_Diameter-mm', 'Branch_Height-cm', 'Branch_Length-cm'};
        branch_excel_filepath = fullfile(output_folder, [tree_id '_branch_trait.xlsx']);
        writetable(T, branch_excel_filepath, 'Sheet', 'Branch_Level_Traits_1')
    end

end
