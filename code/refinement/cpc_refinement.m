function [ori_cpc_optimized_center, ori_cpc_optimized_radius, ori_cpc_optimized_confidence, ori_cpc_optimized_center_outlier, branch_surface_pc] = cpc_refinement(P, i, kdtree, r, maximum_length)
    %% CPC optimization
    %  The assumption for trunk and branch pts is different
    %  For trunk, the initial skeleton pts are always inside the cylinder so it can directly use to retrieve pts
    %  For branch, the initial skelton pts might be outside the cylinder so it has to get the original pts retrieved

    DEBUG = false;

    ori_cpc_optimized_center = [];
    ori_cpc_optimized_radius = [];
    ori_cpc_optimized_confidence = [];
    ori_cpc_optimized_center_outlier = [];

    % retrieve corresponding surface point
    pts1 = P.spls(i, :);
    loc1 = backproject(pts1, P);
    pts1_ori = P.pts(loc1, :);

    % sphere pruning
    original_pt_normalized = P.original_pt;
    x1 = pts1_ori(1); y1 = pts1_ori(2); z1 = pts1_ori(3);
    neighboring_pc_idx = findPointsInROI(original_pt_normalized, [x1 - r, x1 + r, y1 - r, y1 + r, z1 - r, z1 + r]);
    neighboring_pc = select(original_pt_normalized, neighboring_pc_idx);
    branch_surface_pc = neighboring_pc;

    if ~isempty(neighboring_pc.Location)
        % kdtree neighbors
        [knn_index, ~] = knnsearch(kdtree, neighboring_pc.Location, 'K', 50);
        unique_knn_index = unique(knn_index(:));
        branch_surface_pts = original_pt_normalized.Location(unique_knn_index, :);
        branch_surface_pts = [branch_surface_pts; neighboring_pc.Location];
        branch_surface_pts_color = [original_pt_normalized.Color(unique_knn_index, :); neighboring_pc.Color];
        branch_surface_pc = pointCloud(branch_surface_pts, 'Color', branch_surface_pts_color);

        % find branch grow information
        grow_info = find_grow_direction(branch_surface_pc, maximum_length);

        if grow_info.Seg
            %% rotate the point cloud so that the center vector is approximately parallel to XY plane
            % 1st rotation along Z axis and 2nd rotation along either X or Y axis
            % if the branch grows more along X axis
            [x, y, bls, A, B] = align_pts_axis(branch_surface_pts, 1, 2, grow_info);
            tf_surface_pts1 = transformPoint3d(branch_surface_pts, A);

            if grow_info.dimension == 1
                [x2, y2, bls2, A2, B2] = align_pts_axis(tf_surface_pts1, 1, 3, grow_info);
                label = 'x-axis';
            else
                [x2, y2, bls2, A2, B2] = align_pts_axis(tf_surface_pts1, 2, 3, grow_info);
                label = 'y-axis';
            end

            % transform points and find the grow direction (either X or Y axis)
            tf_surface_pts = transformPoint3d(tf_surface_pts1, A2);
            surface_pc = pointCloud(tf_surface_pts);

            % find the transformed branch grow information
            grow_info = find_grow_direction(surface_pc, maximum_length);
            grow_info.maximum_length = maximum_length;

            %% CPC optimization to optimize cylinder center
            % divide the center to small segments
            [cpc_optimized_radius, cpc_optimized_center, cpc_optimized_confidence, segment_inliers] = segment_and_cpc(tf_surface_pts, grow_info);

            if ~isempty(cpc_optimized_center)

                inlier_spls = cpc_optimized_center(segment_inliers == 1, :);
                outlier_spls = cpc_optimized_center(segment_inliers ~= 1, :);
                ori_cpc_optimized_radius = cpc_optimized_radius(segment_inliers == 1);
                ori_cpc_optimized_confidence = cpc_optimized_confidence;

                % transform back to the original coordinates
                inverse_tf = B * B2;
                ori_cpc_optimized_center = transformPoint3d(inlier_spls, inverse_tf);
                ori_cpc_optimized_center_outlier = transformPoint3d(outlier_spls, inverse_tf);

                if DEBUG

                    % visualization purpose
                    r = 3 * r;
                    neighboring_pc_idx = findPointsInROI(original_pt_normalized, [x1 - r, x1 + r, y1 - r, y1 + r, z1 - r, z1 + r]);
                    neighboring_pc = select(original_pt_normalized, neighboring_pc_idx);

                    median_inlier_radius = median(ori_cpc_optimized_radius);

                    ori_cpc_optimized_center_median = median(ori_cpc_optimized_center, 1);

                    figure('Name', 'Branch pts')
                    pcshow(neighboring_pc, 'MarkerSize', 30); hold on
                    set(gcf, 'color', 'white'); set(gca, 'color', 'white')
                    plot3(branch_surface_pts(:, 1), branch_surface_pts(:, 2), branch_surface_pts(:, 3), '.g', 'MarkerSize', 15)
                    plot3(ori_cpc_optimized_center(:, 1), ori_cpc_optimized_center(:, 2), ori_cpc_optimized_center(:, 3), '.r', 'MarkerSize', 30)
                    plot3(ori_cpc_optimized_center_outlier(:, 1), ori_cpc_optimized_center_outlier(:, 2), ori_cpc_optimized_center_outlier(:, 3), '.b', 'MarkerSize', 30)
                    plot3(ori_cpc_optimized_center_median(1), ori_cpc_optimized_center_median(2), ori_cpc_optimized_center_median(3), '.black', 'MarkerSize', 40);
                    fnplt(cscvn(ori_cpc_optimized_center'), 'b', 2)
                    title('Original points', ['Mean distance: ', num2str(median_inlier_radius * 1e3, '%.2f'), ' mm'])
                    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

                end

            else
                ori_cpc_optimized_center = median(branch_surface_pts, 1);
                ori_cpc_optimized_radius = NaN;
                ori_cpc_optimized_confidence = NaN;
            end

        end

    end

end
