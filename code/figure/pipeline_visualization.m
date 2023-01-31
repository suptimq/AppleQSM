clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton'; % folder storing extracted skeleton
exp_id = 'baseline';
extension = '.ply';

files = dir(fullfile(data_folder, ['tree*' extension]));

file = files(3).name;
[filepath, tree_id, ext] = fileparts(file);
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

figure('Name', 'Input')
pcshow(pt, 'MarkerSize', 20);
set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
grid on; axis equal;

figure('Name', 'Skeleton')
pcshow(pt, 'MarkerSize', 10); hold on
set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
scatter3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), 200, '.');
grid on; axis equal;

figure('Name', 'Skeleton connectivity')
scatter3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), 200, '.'); hold on
plot_connectivity(P.spls, P.spls_adj, 2, [1, 0, 0]);
grid on; axis equal;

refined_main_trunk_pts = P.trunk_cpc_optimized_center;


figure('Name', 'Structure segmentation')
plot3(refined_main_trunk_pts(:, 1), refined_main_trunk_pts(:, 2), refined_main_trunk_pts(:, 3), '.r', 'MarkerSize', 30); hold on
start = 0;
colors = {'blue', 'yellow', 'green', 'cyan', 'magenta'};
for i = 1:length(P.primary_center_size)
    primary_branch_pts_size = P.primary_center_size(i);
    index = start + 1:start + primary_branch_pts_size;
    primary_branch_pts = P.primary_branch_center(index, :);
    plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    start = start + primary_branch_pts_size;
end
grid on; axis equal;

figure('Name', 'Refined main trunk')
pcshow(P.trunk_pc, 'MarkerSize', 20); hold on
set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
plot3(refined_main_trunk_pts(:, 1), refined_main_trunk_pts(:, 2), refined_main_trunk_pts(:, 3), '.r', 'MarkerSize', 30);
grid on; axis equal;


xc = P.trunk_diameter_ellipse(1); yc = P.trunk_diameter_ellipse(2); radius_x = P.trunk_diameter_ellipse(3); radius_y = P.trunk_diameter_ellipse(4); theta = P.trunk_diameter_ellipse(5); trunk_radius = P.trunk_radius;
figure('Name', 'Trunk diameter estimation')
scatter(P.trunk_diameter_pts(:, 1), P.trunk_diameter_pts(:, 2), 50, '.', 'r'); hold on
draw_ellipse(radius_x, radius_y, theta, xc, yc, 'k')
grid on; axis equal;

%% bottom primary branch
roi = [-0.2, 0.2, -0.05, 0.18, P.branch_pc.ZLimits(1), P.branch_pc.ZLimits(1)+0.12];
bottom_primary_branch_index = findPointsInROI(P.branch_pc, roi);
bottom_primary_branch_pc = select(P.branch_pc, bottom_primary_branch_index);
roi = [0.05, 0.1, -0.05, 0, P.branch_pc.ZLimits(1), P.branch_pc.ZLimits(1)+0.05];
bottom_primary_branch_root_index = findPointsInROI(P.branch_pc, roi);
bottom_primary_branch_root_pc = select(P.branch_pc, bottom_primary_branch_root_index);
roi = [-0.2, 0.2, -0.05, 0.18, P.trunk_pc.ZLimits(1), P.branch_pc.ZLimits(1)+0.2];
bottom_trunk_index = findPointsInROI(P.trunk_pc, roi);
bottom_trunk_pc = select(P.trunk_pc, bottom_trunk_index);
start = 0;
primary_branch_pts_size = P.primary_center_size(1);
index = start + 1:start + primary_branch_pts_size;
primary_branch_pts = P.primary_branch_center(index, :);
figure('Name', 'Bottom primary branch')
pcshow(bottom_primary_branch_pc, 'MarkerSize', 20); hold on
pcshow(bottom_trunk_pc, 'MarkerSize', 20);
set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
plot3(primary_branch_pts(1:12, 1), primary_branch_pts(1:12, 2), primary_branch_pts(1:12, 3), '.', 'Color', 'yellow', 'MarkerSize', 30);
plot3(refined_main_trunk_pts(1:13, 1), refined_main_trunk_pts(1:13, 2), refined_main_trunk_pts(1:13, 3), '.', 'Color', 'red', 'MarkerSize', 30);

grid on; axis equal;

%% branch diameter estimation
figure('Name', 'Bottom primary branch root')
pcshow(bottom_primary_branch_root_pc, 'MarkerSize', 40); hold on
set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
% plot3(primary_branch_pts(1, 1), primary_branch_pts(1, 2), primary_branch_pts(1, 3), '.', 'Color', 'yellow', 'MarkerSize', 30);
plot3(0.065, -0.01, -0.824, '.', 'Color', 'yellow', 'MarkerSize', 30);

grid on; axis equal;