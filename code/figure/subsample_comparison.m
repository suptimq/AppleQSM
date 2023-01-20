clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton'; % folder storing extracted skeleton
output_folder = 'C:\Users\tq42\Documents\ASABE_2022_Journal';

exp_id = 'baseline';
random_exp_id = 'random_downsample';
grid_exp_id = 'grid_downsample';

extension = '.ply';

files = dir(fullfile(data_folder, ['tree*' extension]));

file = files(3).name;
[filepath, tree_id, ext] = fileparts(file);
skel_filename_format = '_contract_*_skeleton.mat';
skel_filename = search_skeleton_file(tree_id, fullfile(skel_folder, exp_id), skel_filename_format);
skel_filename_random = search_skeleton_file(tree_id, fullfile(skel_folder, random_exp_id), skel_filename_format);
skel_filename_grid = search_skeleton_file(tree_id, fullfile(skel_folder, grid_exp_id), skel_filename_format);

skel_filepath = fullfile(skel_folder, exp_id, skel_filename);
load(skel_filepath, 'P'); 
hc_pt = P;

original_pt_normalized = P.original_pt;
original_pt_normalized_location = original_pt_normalized.Location;
desired_pt = 30000;
ratio = desired_pt / original_pt_normalized.Count;
pt = pcdownsample(original_pt_normalized, 'random', ratio); % visualization purpose only!
pt_location = pt.Location;
pt_color = double(pt.Color) / 255;


skel_filepath = fullfile(skel_folder, random_exp_id, skel_filename_random);
load(skel_filepath, 'P'); 
random_pt = P;

skel_filepath = fullfile(skel_folder, grid_exp_id, skel_filename_grid);
load(skel_filepath, 'P'); 
grid_pt = P;

%% downsampled PC
PC_COLOR = [102, 153, 204] / 255;
figure('Name', 'Downsampled PC comparison')
ax1 = subplot(1, 3, 1);
plot3(hc_pt.pts(:, 1), hc_pt.pts(:, 2), hc_pt.pts(:, 3), '.', 'MarkerEdgeColor', PC_COLOR);
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax2 = subplot(1, 3, 2);
plot3(random_pt.pts(:, 1), random_pt.pts(:, 2), random_pt.pts(:, 3), '.', 'MarkerEdgeColor', PC_COLOR);
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax3 = subplot(1, 3, 3);
plot3(grid_pt.pts(:, 1), grid_pt.pts(:, 2), grid_pt.pts(:, 3), '.', 'MarkerEdgeColor', PC_COLOR);
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

Link = linkprop([ax1, ax2, ax3], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);

%% skeleton
figure('Name', 'Skeleton comparison')
ax1 = subplot(1, 3, 1);
scatter3(pt_location(:, 1), pt_location(:, 2), pt_location(:, 3), 10, pt_color, 'filled', 'o'); hold on
scatter3(hc_pt.spls(:, 1), hc_pt.spls(:, 2), hc_pt.spls(:, 3), 200, '.');
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax2 = subplot(1, 3, 2);
scatter3(pt_location(:, 1), pt_location(:, 2), pt_location(:, 3), 10, pt_color, 'filled', 'o'); hold on
scatter3(random_pt.spls(:, 1), random_pt.spls(:, 2), random_pt.spls(:, 3), 200, '.');
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax3 = subplot(1, 3, 3);
scatter3(pt_location(:, 1), pt_location(:, 2), pt_location(:, 3), 10, pt_color, 'filled', 'o'); hold on
scatter3(grid_pt.spls(:, 1), grid_pt.spls(:, 2), grid_pt.spls(:, 3), 200, '.');
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

Link = linkprop([ax1, ax2, ax3], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);