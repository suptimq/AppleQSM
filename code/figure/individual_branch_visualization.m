function individual_branch_visualization(P, bottom_trunk_pc, bottom_trunk_skeleton_pc, bottom_primary_branch_pc,  mode, figname, colors)

    PC_COLOR = [102, 153, 230] / 255;

    if strcmp(mode, 'primary')
        branch_pts = P.branch_root_pts;
        branch_pts_label = P.branch_root_label;
    elseif strcmp(mode, 'entire')
        branch_pts = P.entire_branch_pts;
        branch_pts_label = P.entire_branch_label;
    end

    [~, tmp_index] = ismember(bottom_primary_branch_pc.Location, branch_pts, 'row');
    bottom_primary_branch_pc_w_label = select(bottom_primary_branch_pc, find(tmp_index ~= 0));
    bottom_primary_branch_pc_label = branch_pts_label(tmp_index(tmp_index ~= 0));
    unique_label = unique(bottom_primary_branch_pc_label);

    figure('Name', figname)
    ax1 = subplot(1, 2, 1);
    pcshow(bottom_trunk_pc, 'MarkerSize', 40); hold on
    set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
    plot3(bottom_trunk_skeleton_pc.Location(:, 1), bottom_trunk_skeleton_pc.Location(:, 2), bottom_trunk_skeleton_pc.Location(:, 3), '.', 'Color', PC_COLOR, 'MarkerSize', 40);
    scatter3(bottom_primary_branch_pc.Location(:, 1), bottom_primary_branch_pc.Location(:, 2), bottom_primary_branch_pc.Location(:, 3), 100, 'filled', 'o');
    grid on; axis equal;

    ax2 = subplot(1, 2, 2);
    pcshow(bottom_trunk_pc, 'MarkerSize', 40); hold on
    set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
    plot3(bottom_trunk_skeleton_pc.Location(:, 1), bottom_trunk_skeleton_pc.Location(:, 2), bottom_trunk_skeleton_pc.Location(:, 3), '.', 'Color', PC_COLOR, 'MarkerSize', 40);

    for i = 1:length(unique_label)
        label = unique_label(i);
        tmp_pc = select(bottom_primary_branch_pc_w_label, bottom_primary_branch_pc_label == label);
        plot3(tmp_pc.Location(:, 1), tmp_pc.Location(:, 2), tmp_pc.Location(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    end

    grid on; axis equal;

    Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
    setappdata(gcf, 'StoreTheLink', Link);

end
