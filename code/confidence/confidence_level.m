function confidence = confidence_level(pts, normal, radius, inliers)
    %% calculate the angle-based confidence
    % 1. create plane based on the center and normal
    % 2. project pts onto the plane
    % 3. count the angle coverage of boundary pts

    % pts: the last row is the center in 3D space

    if nargin < 4
        inliers = 1:size(pts, 1);
    end

    center = pts(end, :);
    % create an orthogonal plane
    plane = createPlane(center, normal);
    projected_points = projPointOnPlane(pts, plane);
    
    % projection
    [meanPoint, V, transformed_pts] = projection_3d(projected_points);
    center2d = transformed_pts(end, 1:2);
    transformed_pts = transformed_pts(1:end-1, 1:2);
    
    % calculate confidence
    x = transformed_pts(:, 1); y = transformed_pts(:, 2);
    x_inliers = x(inliers); y_inliers = y(inliers);
    [nums, ratios] = angle_confidence(center2d, radius, [x_inliers, y_inliers]);
    
    % get results
    ratios = num2cell(ratios); [center_ratio, boundary_ratio, inside_ratio, angle_ratio] = deal(ratios{:});
    nums = num2cell(nums); [num_center_pts, num_near_boundary_pts, num_inside_pts, num_total_pts] = deal(nums{:});
    total_covered_angle = 360 * angle_ratio;
end