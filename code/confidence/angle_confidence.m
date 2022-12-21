function [nums, ratios] = angle_confidence(center2d, radius, pts)

    buffer = radius / 5;
    distance_to_center = pdist2(double(pts), double(center2d));
    near_center_pts_index = distance_to_center <= radius - buffer;
    near_boundary_pts_inside_index = (distance_to_center > radius - buffer) & (distance_to_center <= radius);
    near_boundary_pts_outside_index = (distance_to_center > radius) & (distance_to_center <= radius + buffer);
    
    near_center_pts= pts(near_center_pts_index, :);
    near_boundary_pts_inside = pts(near_boundary_pts_inside_index, :);
    near_boundary_pts_outside = pts(near_boundary_pts_outside_index, :);
    near_boundary_pts = [near_boundary_pts_inside; near_boundary_pts_outside];
   
    num_total_pts = size(pts, 1);
    num_center_pts = size(near_center_pts, 1);
    num_near_boundary_pts = size(near_boundary_pts, 1);
    num_inside_pts = num_center_pts + num_near_boundary_pts;

    boundary_ratio = num_near_boundary_pts / num_total_pts;
    inside_ratio = num_inside_pts / num_total_pts;
    center_ratio = num_center_pts / num_inside_pts;

    %% calculate angle between a vector and the horizontal vector
    bin_width = 20;
    edges = 1:bin_width:361;
     % minimum_pts_within_bin = (num_near_boundary_pts / 360) * (bin_width / 2);
    if center2d(1) > 0
        reference_horizontal_vector = [center2d(1), 0];
    else
        reference_horizontal_vector = [-center2d(1), 0];
    end
    near_boundary_pts_vector = near_boundary_pts - center2d;
    near_boundary_pts_angle = sort(compute_angle(near_boundary_pts_vector, reference_horizontal_vector));
    binned_angle = histcounts(near_boundary_pts_angle, edges);
    total_covered_angle = sum((binned_angle >= 20) * bin_width);
    angle_ratio = total_covered_angle / 360;

    ratios = [center_ratio, boundary_ratio, inside_ratio, angle_ratio];
    nums = [num_center_pts, num_near_boundary_pts, num_inside_pts, num_total_pts];

end