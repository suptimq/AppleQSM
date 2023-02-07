clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13\skeleton'; % folder storing extracted skeleton

exp_id = 'baseline';
random_exp_id = 'random_downsample';
grid_exp_id = 'grid_downsample';

extension = '.ply';

tree_id = 'tree3';
skel_filename_format = '_contract_*_skeleton.mat';
skel_filename = search_skeleton_file(tree_id, fullfile(skel_folder, exp_id), skel_filename_format);
skel_filename_random = search_skeleton_file(tree_id, fullfile(skel_folder, random_exp_id), skel_filename_format);
skel_filename_grid = search_skeleton_file(tree_id, fullfile(skel_folder, grid_exp_id), skel_filename_format);

skel_filepath = fullfile(skel_folder, exp_id, skel_filename);
load(skel_filepath, 'P'); 
hc_pt = P;
hc_pc = pointCloud(hc_pt.pts);
hc_spls_pc = pointCloud(hc_pt.spls);

original_pt_normalized = P.original_pt;

skel_filepath = fullfile(skel_folder, random_exp_id, skel_filename_random);
load(skel_filepath, 'P'); 
random_pt = P;
random_pc = pointCloud(random_pt.pts);
random_spls_pc = pointCloud(random_pt.spls);

skel_filepath = fullfile(skel_folder, grid_exp_id, skel_filename_grid);
load(skel_filepath, 'P'); 
grid_pt = P;
grid_pc = pointCloud(grid_pt.pts);
grid_spls_pc = pointCloud(grid_pt.spls);

%% downsampled PC and skeleton
PC_COLOR = [102, 153, 204] / 255;
figure('Name', 'Downsampled PC comparison')
ax1 = subplot(1, 4, 1);
plot3(original_pt_normalized.Location(:, 1), original_pt_normalized.Location(:, 2), original_pt_normalized.Location(:, 3), '.', 'MarkerEdgeColor', PC_COLOR);
grid on; axis equal;

ax2 = subplot(1, 4, 2);
plot3(hc_pt.pts(:, 1), hc_pt.pts(:, 2), hc_pt.pts(:, 3), '.', 'MarkerEdgeColor', PC_COLOR); hold on
scatter3(hc_pt.spls(:, 1), hc_pt.spls(:, 2), hc_pt.spls(:, 3), 200, '.');
grid on; axis equal;

ax3 = subplot(1, 4, 3);
plot3(random_pt.pts(:, 1), random_pt.pts(:, 2), random_pt.pts(:, 3), '.', 'MarkerEdgeColor', PC_COLOR); hold on
scatter3(random_pt.spls(:, 1), random_pt.spls(:, 2), random_pt.spls(:, 3), 200, '.');
grid on; axis equal;

ax4 = subplot(1, 4, 4);
plot3(grid_pt.pts(:, 1), grid_pt.pts(:, 2), grid_pt.pts(:, 3), '.', 'MarkerEdgeColor', PC_COLOR); hold on
scatter3(grid_pt.spls(:, 1), grid_pt.spls(:, 2), grid_pt.spls(:, 3), 200, '.');
grid on; axis equal;

Link = linkprop([ax1, ax2, ax3], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);

%% zoom-in
roi = [0, 0.5, -0.2, 0.1, -0.7, 0.1];

tmp_index = findPointsInROI(original_pt_normalized, roi);
original_pc_zoom = select(original_pt_normalized, tmp_index);

tmp_index = findPointsInROI(hc_pc, roi);
hc_pc_zoom = select(hc_pc, tmp_index);
tmp_index = findPointsInROI(hc_spls_pc, roi);
hc_spls_pc_zoom = select(hc_spls_pc, tmp_index);

tmp_index = findPointsInROI(random_pc, roi);
random_pc_zoom = select(random_pc, tmp_index);
tmp_index = findPointsInROI(random_spls_pc, roi);
random_spls_pc_zoom = select(random_spls_pc, tmp_index);

tmp_index = findPointsInROI(grid_pc, roi);
grid_pc_zoom = select(grid_pc, tmp_index);
tmp_index = findPointsInROI(grid_spls_pc, roi);
grid_spls_pc_zoom = select(grid_spls_pc, tmp_index);

figure('Name', 'Zoom-in')
ax1 = subplot(1, 4, 1);
scatter3(original_pc_zoom.Location(:, 1), original_pc_zoom.Location(:, 2), original_pc_zoom.Location(:, 3), 5, 'filled', 'o'); hold on
grid on; axis equal;

ax2 = subplot(1, 4, 2);
scatter3(hc_pc_zoom.Location(:, 1), hc_pc_zoom.Location(:, 2), hc_pc_zoom.Location(:, 3), 5, 'filled', 'o'); hold on
scatter3(hc_spls_pc_zoom.Location(:, 1), hc_spls_pc_zoom.Location(:, 2), hc_spls_pc_zoom.Location(:, 3), 280, '.');
grid on; axis equal;

ax3 = subplot(1, 4, 3);
scatter3(random_pc_zoom.Location(:, 1), random_pc_zoom.Location(:, 2), random_pc_zoom.Location(:, 3), 5, 'filled', 'o'); hold on
scatter3(random_spls_pc_zoom.Location(:, 1), random_spls_pc_zoom.Location(:, 2), random_spls_pc_zoom.Location(:, 3), 280, '.');
grid on; axis equal;

ax4 = subplot(1, 4, 4);
scatter3(grid_pc_zoom.Location(:, 1), grid_pc_zoom.Location(:, 2), grid_pc_zoom.Location(:, 3), 5, 'filled', 'o'); hold on
scatter3(grid_spls_pc_zoom.Location(:, 1), grid_spls_pc_zoom.Location(:, 2), grid_spls_pc_zoom.Location(:, 3), 280, '.');
grid on; axis equal;

Link = linkprop([ax1, ax2, ax3, ax4], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);