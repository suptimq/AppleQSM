function [sorted_sliced_main_trunk_pts, row, col] = find_internode(branch_pts, refined_main_trunk_pts, delta, zlimit)

    if nargin < 4
        zlimit = true;
    end

    zmax = max(branch_pts(:, 3));
    zmin = min(branch_pts(:, 3));

    main_trunk_pts_idx = refined_main_trunk_pts(:, 3) >= zmin - delta & refined_main_trunk_pts(:, 3) <= zmax + delta;
    sliced_main_trunk_pts = refined_main_trunk_pts(main_trunk_pts_idx, :);
    sorted_sliced_main_trunk_pts = sortrows(sliced_main_trunk_pts, 3);
    distance_matrix = pdist2(sorted_sliced_main_trunk_pts, branch_pts); % each row of distance matrix represents the distance between a main trunk point to all branch points
    [~, idx] = min(distance_matrix(:));
    [row, col] = ind2sub(size(distance_matrix), idx); % row and col represent the main trunk and branch point, respectively

    % make sure trunk internode's Z value <= first two primary branch
    % skeleton points
    if zlimit && ~isempty(row)
        branch_branch_distance_matrix = pdist2(branch_pts, branch_pts);
        [~, index] = mink(branch_branch_distance_matrix(col, :), 2);
        zmin = min(branch_pts(index, 3));
        trunk_internode = sliced_main_trunk_pts(row, :);
        
        while row > 1 && trunk_internode(3) > zmin + 0.01
            row = row - 1;
            trunk_internode = sliced_main_trunk_pts(row, :);
        end
    end
end