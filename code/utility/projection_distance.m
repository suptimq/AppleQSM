function [intersection_pts, nearest_pts, point_point_distance, line_line_distance] = projection_distance(v1, v2)
    %% compute the length of the extend line
    %% 1. find the intersection point of two lines
    %% 2. calculate the distance to the nearest point
    %% Return
    %% point_point_distance - point-to-point distance
    %% line_line_distance - line-to-line distance

    A = [v1(1, :); v2(1, :)];
    B = [v1(2, :); v2(2, :)];

    [intersection_pts, nearest_pts, distance_to_intersection, ~, ~, ~] = lineXline(A, B);
    point_point_distance = pdist2(nearest_pts(2, :), v1(1, :));
    line_line_distance = pdist(nearest_pts);
end