clc; clear; close all;
path('..', path)

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton\multiplier_by_3_cpc_sphere_radius_002'; % folder storing extracted skeleton

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
ax1 = subplot(1, 2, 1);
pcshow(P.branch_pc, 'Markersize', 30); hold on
plot3(P.primary_branch_center(:, 1), P.primary_branch_center(:, 2), P.primary_branch_center(:, 3), '.r', 'Markersize', 30)
plot3(P.side_branch_center(:, 1), P.side_branch_center(:, 2), P.side_branch_center(:, 3), '.b', 'Markersize', 30)
plot3(invalid_centers(:, 1), invalid_centers(:, 2), invalid_centers(:, 3), '.g', 'Markersize', 30)
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); axis equal; grid on;

ax2 = subplot(1, 2, 2);
pcshow(P.branch_pc, 'Markersize', 30); hold on
plot_by_weight(valid_centers, valid_radii / P.trunk_radius);
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); axis equal; grid on;