function [cpc_optimzed_center, cpc_optimized_radius, cpc_optimized_confidence, trunk_pc] = trunk_refinement(P, main_trunk_pts_idx, options)
    %% Trunk refinement

    cpc_optimzed_center = [];
    cpc_optimized_radius = [];
    cpc_optimized_confidence = [];

    %%  visualization purpose only and show the original point clouds
    original_pt_normalized = P.original_pt;

    %% define ellisoid pruning parameters
    xy_radius = options.xy_radius;                   % 0.035
    z_radius = options.z_radius;                     % 0.02
    maximum_length = options.maximum_length;         % 0.005 cross-section thickness

    optimized_spls = [];
    optimized_radius = [];
    optimized_confidence = [];
    trunk_pc = [];

    for i = 1:length(main_trunk_pts_idx)

        pts1 = P.spls(main_trunk_pts_idx(i), :);
        x1 = pts1(1); y1 = pts1(2); z1 = pts1(3);

        neighboring_pts_idx = findPointsInROI(original_pt_normalized, [x1 - xy_radius, x1 + xy_radius, y1 - xy_radius, y1 + xy_radius, z1 - z_radius, z1 + z_radius]);
        neighboring_pc = select(original_pt_normalized, neighboring_pts_idx);
        trunk_pc = [trunk_pc; neighboring_pc];

        if ~isempty(neighboring_pc.Location)
            zmin = neighboring_pc.ZLimits(1);
            zmax = neighboring_pc.ZLimits(2);

            %% run CPC on each small trunk segment
            start = zmin;
            tmp_center = [];
            tmp_radius = [];
            tmp_confidence = [];

            while start < zmax

                index = (neighboring_pc.Location(:, 3) >= start) & (neighboring_pc.Location(:, 3) <= start + maximum_length);
                in_between_pts = neighboring_pc.Location(index, :);

                if size(in_between_pts, 1) > 1
                    center = cpc_optimization(in_between_pts);
                    radius = mean(pdist2(double(center), double(in_between_pts)));

                    [nums, ratios] = angle_confidence(center(1:2), radius, neighboring_pc.Location(:, 1:2));
                    ratios = num2cell(ratios);
                    [center_ratio, boundary_ratio, inside_ratio, angle_ratio] = deal(ratios{:});
                    
                    tmp_center = [tmp_center; center];
                    tmp_radius = [tmp_radius; radius];
                    tmp_confidence = [tmp_confidence; angle_ratio];
                end

                start = start + maximum_length;
            end

            % 1st local RANSAC to remove outliers
            min_samples = 3;
            residual_threshold = maximum_length;
            max_trials = 1e3;

            if size(tmp_center, 1) > min_samples * 2
                [~, segment_inliers, ~] = ransac_py(tmp_center, '3D_Line', min_samples, residual_threshold, max_trials);
            else
                segment_inliers = ones(size(tmp_center, 1), 1);
            end
            
                inlier_spls = tmp_center(segment_inliers == 1, :);
                inlier_radius = tmp_radius(segment_inliers == 1, :);
                inlier_confidence = tmp_confidence(segment_inliers == 1, :);
                median_inlier_spls = median(inlier_spls, 1);
                median_inlier_radius = median(inlier_radius, 1);
                median_inlier_confidence = median(inlier_confidence, 1);
    
                optimized_spls = [optimized_spls; median_inlier_spls];
                optimized_radius = [optimized_radius; median_inlier_radius];
                optimized_confidence = [optimized_confidence; median_inlier_confidence];

        end

    end

    %% 2nd semi-global RANSAC to remove outliers
    N = options.N; % 10
    start = 0;
    ransac_inlier_spls = [];
    ransac_inlier_radius = [];
    ransac_inlier_confidence = [];
    ransac_outlier_spls = [];

    while start <= size(optimized_spls, 1)

        if start + N <= size(optimized_spls, 1)
            index = start + 1:start + N;
            min_samples = 6;
        else
            index = start + 1:size(optimized_spls, 1);
            min_samples = 3;
        end

        % RANSAC parameters
        residual_threshold = maximum_length;
        max_trials = 1e3;

        % select small segment
        segment = optimized_spls(index, :);
        segment_radius = optimized_radius(index, :);
        segment_confidence = optimized_confidence(index);

        if size(segment, 1) > min_samples
            [~, segment_inliers, ~] = ransac_py(segment, '3D_Line', min_samples, residual_threshold, max_trials);

            ransac_inlier_spls = [ransac_inlier_spls; segment(segment_inliers == 1, :)];
            ransac_inlier_radius = [ransac_inlier_radius; segment_radius(segment_inliers == 1)];
            ransac_inlier_confidence = [ransac_inlier_confidence; segment_confidence(segment_inliers == 1)];
            ransac_outlier_spls = [ransac_outlier_spls; segment(segment_inliers ~= 1, :)];
        else
            ransac_inlier_spls = [ransac_inlier_spls; segment];
            ransac_inlier_radius = [ransac_inlier_radius; segment_radius];
            ransac_inlier_confidence = [ransac_inlier_confidence; segment_confidence];
        end

        start = start + N;
    end

    %% fit a spline and uniformly sample points from the spline
    trunk_pc = pccat(trunk_pc);
    trunk_pc_zmin = trunk_pc.ZLimits(1);
    trunk_pc_zmax = trunk_pc.ZLimits(2);

    M = options.M; % 100 #sample
    [curve, uniform_xyz] = spline_interpolation(ransac_inlier_spls, M);

    %% KNN for density
    sampled_radius = [];
    sampled_confidence = [];
    kdtree_spls = KDTreeSearcher(ransac_inlier_spls);

    for i = 1:size(uniform_xyz, 1)
        sampled_spls = uniform_xyz(i, :);
        [index, ~] = knnsearch(kdtree_spls, sampled_spls, 'K', 3);
        sampled_radius = [sampled_radius, mean(ransac_inlier_radius(index))];
        sampled_confidence = [sampled_confidence, mean(ransac_inlier_confidence(index))];
    end

    cpc_optimzed_center = uniform_xyz;
    cpc_optimized_radius = sampled_radius;
    cpc_optimized_confidence = sampled_confidence;
end
