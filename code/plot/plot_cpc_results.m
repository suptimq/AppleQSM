clc; clear; close all;
path('..', path)

skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13\segmentation\hc_downsample_iter_7\s1'; % folder storing extracted skeleton

tree_id = 'tree1';
skel_filename_format = '_contract_*_skeleton.mat';
skel_filename = search_skeleton_file(tree_id, skel_folder, skel_filename_format);

skel_filepath = fullfile(skel_folder, skel_filename);
load(skel_filepath, 'P'); % P results from skeleton operation

figure('Name', 'Optimized skeleton pts')
ax1 = subplot(1, 2, 1);
pcshow(P.trunk_pc, 'MarkerSize', 30); hold on
plot3(P.trunk_cpc_optimized_center(:, 1), P.trunk_cpc_optimized_center(:, 2), P.trunk_cpc_optimized_center(:, 3), '.r', 'MarkerSize', 30)
title('Skeleton pts', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax2 = subplot(1, 2, 2);
plot_by_weight(P.trunk_cpc_optimized_center, P.trunk_cpc_optimized_radius / 35)
title('Skeleton pts radius', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);


%% visualization of branch CPC results
invalid_primary_branch_index = isnan(P.primary_branch_radius);
invalid_side_branch_index = isnan(P.side_branch_radius);
valid_centers = [P.primary_branch_center(~invalid_primary_branch_index, :); P.side_branch_center(~invalid_side_branch_index, :)];
valid_radii = [P.primary_branch_radius(~invalid_primary_branch_index); P.side_branch_radius(~invalid_side_branch_index)];

invalid_centers = [P.primary_branch_center(invalid_primary_branch_index, :); P.side_branch_center(invalid_side_branch_index, :)];
invalid_radii = [P.primary_branch_radius(invalid_primary_branch_index); P.side_branch_radius(invalid_side_branch_index)];

figure('Name', 'Optimized branch skeleton pts')
pcshow(P.branch_pc, 'Markersize', 30); hold on
plot3(P.primary_branch_center(:, 1), P.primary_branch_center(:, 2), P.primary_branch_center(:, 3), '.r', 'Markersize', 30)
plot3(P.side_branch_center(:, 1), P.side_branch_center(:, 2), P.side_branch_center(:, 3), '.b', 'Markersize', 30)
plot3(invalid_centers(:, 1), invalid_centers(:, 2), invalid_centers(:, 3), '.g', 'Markersize', 30)
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); axis equal; grid on;

figure('Name', 'Optimized branch skeleton pts w/ weight')
pcshow(P.branch_pc, 'Markersize', 30); hold on
plot_by_weight(valid_centers, valid_radii / P.trunk_radius);
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); axis equal; grid on;

%%
start = 0;
spline_start = 0;
figure('Name', 'Colored branches')
pcshow(P.original_pt, 'MarkerSize', 30); hold on
colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta', 'white'};
color_code = {[1, 0, 0], [0, 0, 1], [1, 1, 0], [0, 1, 0], [0, 1, 1], [1, 0, 1], [1, 1, 1]};

new_branch_location = [];
new_branch_color = [];

for i = 1:P.branch_counter
    primary_branch_pts_size = P.primary_center_size(i);
    index = start + 1:start + primary_branch_pts_size;
    primary_branch_pts = P.primary_branch_center(index, :);

    primary_spline_pts_size = P.primary_spline_size(i);
    index = spline_start + 1:spline_start + primary_spline_pts_size;
    primary_spline_pts = P.primary_spline_center(index, :);
    
    ci = rem(i, length(colors)) + 1;
    new_branch_location = [new_branch_location; primary_branch_pts];
    new_branch_color = [new_branch_color; repmat(color_code{ci}*255, size(primary_branch_pts, 1), 1)];

    plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.', 'Color', colors{ci}, 'MarkerSize', 20);

    start = start + primary_branch_pts_size;
    spline_start = spline_start + primary_spline_pts_size;
end

colored_branch_pc = pointCloud(new_branch_location, 'Color', new_branch_color);
figure('Name', 'Validation');
pcshow(P.original_pt, 'MarkerSize', 30); hold on
pcshow(colored_branch_pc, 'MarkerSize', 200);
axis off;

% filepath = ['D:\Data\colored_branch_' tree_id];
% pcwrite(colored_branch_pc, filepath, PLYFormat='binary');