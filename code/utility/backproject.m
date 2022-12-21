function loc = backproject(pts, P)
    %% find the index of pts w.r.t the contracted points (i.e., P.cpts)

    [~, loc] = ismember(pts, P.cpts, 'row');
    loc_empty_idx = find(loc == 0);

    % if the point was updated in collapse_edge (i.e., loc_empty_idx is not empty)
    % then first retrieve the parent points of the current point
    if ~isempty(loc_empty_idx)
        loc_empty_pts = pts(loc_empty_idx, :);
        [~, loc2] = ismember(loc_empty_pts, P.removed_spls, 'row');
        assert(isempty(find(loc2 == 0, 1)), 'back-projection ERROR');
        tmp_pts = P.removed_spls(loc2 - 1, :);
        tmp_pts2 = P.removed_spls(loc2 - 2, :);
        [~, loc3] = ismember(tmp_pts, P.cpts, 'row');
        [~, loc4] = ismember(tmp_pts2, P.cpts, 'row');
        loc3_empty = find(loc3 == 0);
        loc3(loc3_empty) = loc4(loc3_empty);
        loc(loc_empty_idx) = loc3;
        loc = loc(loc ~= 0);

    end

end
