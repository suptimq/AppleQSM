clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\segmentation'; % folder storing extracted skeleton
exp_id = 'multiplier_by_3_cpc_sphere_radius_002';
compare_exp_id = 'plain_mst';
extension = '.ply';

files = dir(fullfile(data_folder, ['tree*' extension]));

id = 5;
file = files(id).name;
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

%%
PC_COLOR = [102, 153, 230] / 255;
refined_main_trunk_pts = P.trunk_cpc_optimized_center;
refined_main_trunk_pc = pointCloud(refined_main_trunk_pts);
coarse_branch_pts = P.all_branch_pts;
coarse_branch_pc = pointCloud(coarse_branch_pts);

figure
pcshow(pt, 'MarkerSize', 20); hold on
set(gcf, 'Color', 'white'); set(gca, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
plot3(refined_main_trunk_pts(:, 1), refined_main_trunk_pts(:, 2), refined_main_trunk_pts(:, 3), '.', 'Color', PC_COLOR, 'MarkerSize', 20);
scatter3(coarse_branch_pts(:, 1), coarse_branch_pts(:, 2), coarse_branch_pts(:, 3), 20, 'filled', 'o');
grid on; axis equal;

%%
figure
pcshow(pt, 'MarkerSize', 20); hold on
set(gcf, 'Color', 'white'); set(gca, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
plot_dbscan_clusters(P.branch_root_pts, P.branch_root_label);
grid on; axis equal;

%%
start = 0;
side_start = 0;
colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
figure
pcshow(pt, 'MarkerSize', 20); hold on
set(gcf, 'Color', 'white'); 
set(gca, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
for i = 1:length(P.primary_center_size)
    primary_branch_pts_size = P.primary_center_size(i);
    side_branch_pts_size = P.side_center_size(i);

    index = start + 1:start + primary_branch_pts_size;
    side_index = side_start + 1:side_start + side_branch_pts_size;

    primary_branch_pts = P.primary_branch_center(index, :);
    side_branch_pts = P.side_branch_center(side_index, :);

    plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    plot3(side_branch_pts(:, 1), side_branch_pts(:, 2), side_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    text(primary_branch_pts(1, 1), primary_branch_pts(1, 2), primary_branch_pts(1, 3) + 0.03, num2str(i), 'Color', 'red', 'HorizontalAlignment', 'left', 'FontSize', 12);

    start = start + primary_branch_pts_size;
    side_start = side_start + side_branch_pts_size;
end
grid on; axis equal;

%%
start = 0;
colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
figure
pcshow(pt, 'MarkerSize', 20); hold on
set(gcf, 'Color', 'white'); set(gca, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
for i = 1:length(P.primary_center_size)
    primary_branch_pts_size = P.primary_center_size(i);
    index = start + 1:start + primary_branch_pts_size;
    primary_branch_pts = P.primary_branch_center(index, :);
    plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    start = start + primary_branch_pts_size;
end
grid on; axis equal;

%% individual branches (missing branch root point)
if id == 5
    roi = [-0.15, 0.4, -0.25, 0.4, -1, -0.65];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);
    
    roi = [-0.15, 0.4, -0.25, 0.4, -1, -0.65];
    bottom_trunk_index = findPointsInROI(refined_main_trunk_pc, roi);
    bottom_trunk_skeleton_pc = select(refined_main_trunk_pc, bottom_trunk_index);
    
    roi = [-0.15, 0.4, -0.25, 0.4, -1, -0.65];
    bottom_primary_branch_index = findPointsInROI(coarse_branch_pc, roi);
    bottom_primary_branch_pc = select(coarse_branch_pc, bottom_primary_branch_index);
    
    [~, tmp_index] = ismember(bottom_primary_branch_pc.Location, P.branch_root_pts, 'row');
    bottom_primary_branch_pc_w_label = select(bottom_primary_branch_pc, find(tmp_index~=0));
    bottom_primary_branch_pc_label = P.branch_root_label(tmp_index(tmp_index~=0));
    unique_label = unique(bottom_primary_branch_pc_label);

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
    figure('Name', 'Bottom primary branch 1')
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
        tmp_pc = select(bottom_primary_branch_pc_w_label, bottom_primary_branch_pc_label==label);
        plot3(tmp_pc.Location(:, 1), tmp_pc.Location(:, 2), tmp_pc.Location(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    end
    grid on; axis equal;
    
    % individual branches (one node two branches)
    roi = [0, 0.14, -0.2, 0.03, -0.5, -0.4];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);
    
    roi = [0, 0.14, -0.2, 0.03, -0.5, -0.4];
    bottom_trunk_index = findPointsInROI(refined_main_trunk_pc, roi);
    bottom_trunk_skeleton_pc = select(refined_main_trunk_pc, bottom_trunk_index);
    
    roi = [0, 0.14, -0.2, 0.03, -0.5, -0.4];
    bottom_primary_branch_index = findPointsInROI(coarse_branch_pc, roi);
    bottom_primary_branch_pc = select(coarse_branch_pc, bottom_primary_branch_index);
    
    figure('Name', 'Bottom primary branch 2')
    pcshow(bottom_trunk_pc, 'MarkerSize', 40); hold on
    set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
    plot3(bottom_trunk_skeleton_pc.Location(:, 1), bottom_trunk_skeleton_pc.Location(:, 2), bottom_trunk_skeleton_pc.Location(:, 3), '.', 'Color', PC_COLOR, 'MarkerSize', 40);
    scatter3(bottom_primary_branch_pc.Location(:, 1), bottom_primary_branch_pc.Location(:, 2), bottom_primary_branch_pc.Location(:, 3), 100, 'filled', 'o');
    grid on; axis equal;
elseif id == 8
    roi = [-0.1, 0.1, 0, 0.25, -0.25, -0.16];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);
    
    roi = [-0.1, 0.1, 0, 0.25, -0.25, -0.16];
    bottom_trunk_index = findPointsInROI(refined_main_trunk_pc, roi);
    bottom_trunk_skeleton_pc = select(refined_main_trunk_pc, bottom_trunk_index);
    
    roi = [-0.1, 0.1, 0, 0.25, -0.25, -0.16];
    bottom_primary_branch_index = findPointsInROI(coarse_branch_pc, roi);
    bottom_primary_branch_pc = select(coarse_branch_pc, bottom_primary_branch_index);
    
    figure('Name', 'Bottom primary branch 1')
    pcshow(bottom_trunk_pc, 'MarkerSize', 40); hold on
    set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
    plot3(bottom_trunk_skeleton_pc.Location(:, 1), bottom_trunk_skeleton_pc.Location(:, 2), bottom_trunk_skeleton_pc.Location(:, 3), '.', 'Color', PC_COLOR, 'MarkerSize', 40);
    scatter3(bottom_primary_branch_pc.Location(:, 1), bottom_primary_branch_pc.Location(:, 2), bottom_primary_branch_pc.Location(:, 3), 100, 'filled', 'o');
    grid on; axis equal;
end
%%