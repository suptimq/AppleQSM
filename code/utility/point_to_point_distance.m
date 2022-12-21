function distance_list = point_to_point_distance(pts)
    %% find the distance between each two neighboring points in an array

    distance_list = zeros(1, size(pts, 1) - 1);

    for i = 1:size(pts, 1) - 1
        distance_ = pdist([pts(i); pts(i + 1)]);
        distance_list(i) = distance_;
    end

end
