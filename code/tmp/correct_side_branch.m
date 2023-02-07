clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13\segmentation'; % folder storing extracted skeleton
exp_id = 'multiplier_by_3_cpc_sphere_radius_002';
compare_exp_id = 'plain_mst';
extension = '.ply';

tree_id = 'tree4';
skel_filename_format = '_contract_*_skeleton.mat';
skel_filename = search_skeleton_file(tree_id, fullfile(skel_folder, exp_id), skel_filename_format);

skel_filepath = fullfile(skel_folder, exp_id, skel_filename);
load(skel_filepath, 'P'); % P results from skeleton operation

% visualization purpose only and show the original point clouds
original_pt_normalized = P.original_pt;
original_pt_normalized_location = original_pt_normalized.Location;
desired_pt = 30000;
ratio = desired_pt / original_pt_normalized.Count;
pt = pcdownsample(original_pt_normalized, 'random', ratio); % visualization purpose only!

start = 0;
side_start = 0;
colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
figure('Name', 'Entire branch')
pcshow(pt, 'MarkerSize', 20); hold on
set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
for i = 1:length(P.primary_center_size)
    primary_branch_pts_size = P.primary_center_size(i);
    side_branch_pts_size = P.side_center_size(i);
    index = start + 1:start + primary_branch_pts_size;
    side_index = side_start + 1:side_start + side_branch_pts_size;
    primary_branch_pts = P.primary_branch_center(index, :);
    side_branch_pts = P.side_branch_center(side_index, :);

    tmp_pts = primary_branch_pts(1, :);
    plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    plot3(side_branch_pts(:, 1), side_branch_pts(:, 2), side_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    text(tmp_pts(1), tmp_pts(2), tmp_pts(3) + 0.02, num2str(i), 'Color', 'red', 'HorizontalAlignment', 'left', 'FontSize', 12);

    start = start + primary_branch_pts_size;
    side_start = side_start + side_branch_pts_size;
end
grid on; axis equal;