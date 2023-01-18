function crotch_pts_idx = sphere_pruning(spls, main_trunk_pts, main_trunk_pts_idx, search_radius)

    distance_matrix = pdist2(double(main_trunk_pts), double(spls));
    pts_inside_sphere = distance_matrix < search_radius;

    [~, pts_index_inside_sphere] = ind2sub(size(distance_matrix), find(pts_inside_sphere));
    crotch_pts_idx = setdiff(pts_index_inside_sphere, main_trunk_pts_idx);

end
