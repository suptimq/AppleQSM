function crotch_pts_idx = sphere_pruning(P, main_trunk_pts_idx, search_radius)

    spls = P.spls;
    main_trunk_pts_sorted = P.trunk_cpc_optimized_center;

    distance_matrix = pdist2(double(main_trunk_pts_sorted), double(spls));
    pts_inside_sphere = distance_matrix < search_radius;

    [~, pts_index_inside_sphere] = ind2sub(size(distance_matrix), find(pts_inside_sphere));
    crotch_pts_idx = setdiff(pts_index_inside_sphere, main_trunk_pts_idx);

end
