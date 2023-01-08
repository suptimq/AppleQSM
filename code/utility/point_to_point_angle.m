function [pts_vector, pts_vector_angle] = point_to_point_angle(pts)
    %% find the angle between each two neighboring vectors formed by two neighboring points in an array

    pts_vector = [];
    for k = 1:size(pts, 1)-1
        tmp_vector = pts(k+1, :) - pts(k, :);
        pts_vector = [pts_vector; tmp_vector];
    end
    
    pts_vector_angle = [];
    for k = 1:size(pts_vector, 1)-1
        tmp_angle = compute_angle(pts_vector(k, :), pts_vector(k+1, :));
        pts_vector_angle = [pts_vector_angle, tmp_angle];
    end

end