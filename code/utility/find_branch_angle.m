function [segment_vectors, vertical_angle, N] = find_branch_angle(trunk_internode, trunk_radius, trunk_vector, primary_spline_pts, N)

    % filter out branch skeleton points that are within trunk
    uniform_xyz_distance = pdist2(primary_spline_pts, double(trunk_internode));
    index = uniform_xyz_distance >= trunk_radius;
    primary_spline_pts_outside = primary_spline_pts(index, :);

    % fit branch segment vector
    min_samples = 3; residual_threshold = 0.015; max_trials = 1e3;
    segment_pts = sliding_window(primary_spline_pts_outside, 1, 4, 1);
    num_segment = size(segment_pts, 1);
    segment_vectors = zeros(num_segment, 6);

    if num_segment < N
        N = num_segment;
    end

    for j = 1:N
        [segment_vector, ~, ~] = ransac_py(segment_pts{j}, '3D_Line', min_samples, residual_threshold, max_trials);
        segment_vectors(j, :) = segment_vector;
    end

    % direction check
    branch_grow_direction = primary_spline_pts_outside(3, :) - primary_spline_pts_outside(2, :);
    branch_grow_direction2 = primary_spline_pts_outside(2, :) - primary_spline_pts_outside(1, :);

    for j = 1:N
        segment_direction = segment_vectors(j, 4:end);

        if sum(sign(branch_grow_direction) == sign(segment_direction)) < 2 && sum(sign(branch_grow_direction2) == sign(segment_direction)) < 2
            segment_vectors(j, 4:end) = -segment_vectors(j, 4:end);
        end

    end

    % angle
    vertical_angle_list = [];

    for j = 1:N
        vertical_angle_list = [vertical_angle_list, compute_angle(trunk_vector(4:end), segment_vectors(j, 4:end))];
    end

    vertical_angle = median(vertical_angle_list);

    %% the following code implements the idea of finding the intersection of 
    %% the branch vector and trunk vector
    % if num > 3
    %     ttt = 1:4;
    %     min_samples = 3;
    % else
    %     ttt = 1:num;
    %     min_samples = num - 1;
    % end
    % residual_threshold = 0.015; max_trials = 1e3;
    % [v2, ~, ~] = ransac_py(primary_branch_pts(ttt, :), '3D_Line', min_samples, residual_threshold, max_trials);
    % A = [v2(1:3); v1(1:3)];
    % B = [v2(1:3)+0.5*v2(4:6); v1(1:3) + 0.5*v1(4:6)];
    % [intersection_pts, nearest_pts, distance_to_intersection, ~, ~, ~] = lineXline(A, B);
    % distance_matrix = pdist2(double(nearest_pts(2, :)), sliced_main_trunk_pts);
    % [~, tmp_index] = min(distance_matrix);
    % trunk_internode_2 = sliced_main_trunk_pts(tmp_index, :);
    % tmp_pts_list = [trunk_internode_2; primary_branch_pts];
    % [curve_2, primary_spline_pts_2] = spline_interpolation(double(tmp_pts_list), size(primary_spline_pts, 1)); 
end