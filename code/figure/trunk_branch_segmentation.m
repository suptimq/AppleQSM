clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13\segmentation'; % folder storing extracted skeleton
exp_id = 'hc_downsample_iter_7';
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

%%
PC_COLOR = [102, 153, 230] / 255;
refined_main_trunk_pts = P.trunk_cpc_optimized_center;
refined_main_trunk_pc = pointCloud(refined_main_trunk_pts);
coarse_branch_pts = P.all_branch_pts;
coarse_branch_pc = pointCloud(coarse_branch_pts);

figure
ax1 = subplot(1, 4, 1);
plot3(original_pt_normalized.Location(:, 1), original_pt_normalized.Location(:, 2), original_pt_normalized.Location(:, 3), '.', 'MarkerSize', 5, 'MarkerEdgeColor', PC_COLOR); hold on
plot3(refined_main_trunk_pts(:, 1), refined_main_trunk_pts(:, 2), refined_main_trunk_pts(:, 3), '.', 'Color', 'red', 'MarkerSize', 20);
scatter3(coarse_branch_pts(:, 1), coarse_branch_pts(:, 2), coarse_branch_pts(:, 3), 20, 'filled', 'o');
grid on; axis equal;

%%
ax2 = subplot(1, 4, 2);
plot3(original_pt_normalized.Location(:, 1), original_pt_normalized.Location(:, 2), original_pt_normalized.Location(:, 3), '.', 'MarkerSize', 5, 'MarkerEdgeColor', PC_COLOR); hold on
plot_dbscan_clusters(P.branch_root_pts, P.branch_root_label);
grid on; axis equal;

%%
start = 0;
side_start = 0;
colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
ax3 = subplot(1, 4, 3);
plot3(original_pt_normalized.Location(:, 1), original_pt_normalized.Location(:, 2), original_pt_normalized.Location(:, 3), '.', 'MarkerSize', 5, 'MarkerEdgeColor', PC_COLOR); hold on

for i = 1:length(P.primary_center_size)
    primary_branch_pts_size = P.primary_center_size(i);
    side_branch_pts_size = P.side_center_size(i);

    index = start + 1:start + primary_branch_pts_size;
    side_index = side_start + 1:side_start + side_branch_pts_size;

    primary_branch_pts = P.primary_branch_center(index, :);
    side_branch_pts = P.side_branch_center(side_index, :);

    plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    plot3(side_branch_pts(:, 1), side_branch_pts(:, 2), side_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    % text(primary_branch_pts(1, 1), primary_branch_pts(1, 2), primary_branch_pts(1, 3) + 0.03, num2str(i), 'Color', 'red', 'HorizontalAlignment', 'left', 'FontSize', 12);

    start = start + primary_branch_pts_size;
    side_start = side_start + side_branch_pts_size;
end

grid on; axis equal;

%%
start = 0;
colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
ax4 = subplot(1, 4, 4);
plot3(original_pt_normalized.Location(:, 1), original_pt_normalized.Location(:, 2), original_pt_normalized.Location(:, 3), '.', 'MarkerSize', 5, 'MarkerEdgeColor', PC_COLOR); hold on

for i = 1:length(P.primary_center_size)
    primary_branch_pts_size = P.primary_center_size(i);
    index = start + 1:start + primary_branch_pts_size;
    primary_branch_pts = P.primary_branch_center(index, :);
    plot3(primary_branch_pts(:, 1), primary_branch_pts(:, 2), primary_branch_pts(:, 3), '.', 'Color', colors{rem(i, length(colors)) + 1}, 'MarkerSize', 30);
    start = start + primary_branch_pts_size;
end

grid on; axis equal;

Link = linkprop([ax1, ax2, ax3, ax4], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);

%% individual branches (missing branch root point)
if strcmp(tree_id, 'tree5')
    roi = [-0.15, 0.4, -0.25, 0.4, -1, -0.65];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    roi = [-0.15, 0.4, -0.25, 0.4, -1, -0.65];
    bottom_trunk_index = findPointsInROI(refined_main_trunk_pc, roi);
    bottom_trunk_skeleton_pc = select(refined_main_trunk_pc, bottom_trunk_index);

    roi = [-0.15, 0.4, -0.25, 0.4, -1, -0.65];
    bottom_primary_branch_index = findPointsInROI(coarse_branch_pc, roi);
    bottom_primary_branch_pc = select(coarse_branch_pc, bottom_primary_branch_index);

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
    figname1 = 'Bottom primary branch 1';
    individual_branch_visualization(P, bottom_trunk_pc, bottom_trunk_skeleton_pc, bottom_primary_branch_pc, 'primary', figname1, colors);

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

    colors = {'red', 'magenta', 'blue', 'yellow'};
    figname2 = 'Bottom primary branch 2';
    individual_branch_visualization(P, bottom_trunk_pc, bottom_trunk_skeleton_pc, bottom_primary_branch_pc, 'primary', figname2, colors);

elseif strcmp(tree_id, 'tree8')
    roi = [-0.1, 0.1, -0.08, 0.2, -0.25, -0.05];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    roi = [-0.1, 0.1, -0.08, 0.2, -0.25, -0.05];
    bottom_trunk_index = findPointsInROI(refined_main_trunk_pc, roi);
    bottom_trunk_skeleton_pc = select(refined_main_trunk_pc, bottom_trunk_index);

    roi = [-0.1, 0.1, -0.08, 0.2, -0.25, -0.05];
    bottom_primary_branch_index = findPointsInROI(coarse_branch_pc, roi);
    bottom_primary_branch_pc = select(coarse_branch_pc, bottom_primary_branch_index);

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
    figname1 = 'Bottom primary branch 1';
    individual_branch_visualization(P, bottom_trunk_pc, bottom_trunk_skeleton_pc, bottom_primary_branch_pc, 'primary', figname1, colors);

    roi = [-0.08, 0.06, -0.1, 0.13, -0.76, -0.6];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    roi = [-0.08, 0.06, -0.1, 0.13, -0.76, -0.6];
    bottom_trunk_index = findPointsInROI(refined_main_trunk_pc, roi);
    bottom_trunk_skeleton_pc = select(refined_main_trunk_pc, bottom_trunk_index);

    roi = [-0.08, 0.06, -0.1, 0.13, -0.76, -0.6];
    bottom_primary_branch_index = findPointsInROI(coarse_branch_pc, roi);
    bottom_primary_branch_pc = select(coarse_branch_pc, bottom_primary_branch_index);

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
    figname1 = 'Bottom primary branch 2';
    individual_branch_visualization(P, bottom_trunk_pc, bottom_trunk_skeleton_pc, bottom_primary_branch_pc, 'primary', figname1, colors);
elseif strcmp(tree_id, 'tree1')
    roi = [-0.1, 0.54, -0.25, 0.1, -0.4, -0.1];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    roi = [-0.1, 0.54, -0.25, 0.1, -0.4, -0.1];
    bottom_trunk_index = findPointsInROI(refined_main_trunk_pc, roi);
    bottom_trunk_skeleton_pc = select(refined_main_trunk_pc, bottom_trunk_index);

    roi = [-0.05, 0.54, -0.25, 0.08, -0.4, -0.1];
    bottom_primary_branch_index = findPointsInROI(coarse_branch_pc, roi);
    bottom_primary_branch_pc = select(coarse_branch_pc, bottom_primary_branch_index);

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
    figname1 = 'Bottom primary branch 1';
    individual_branch_visualization(P, bottom_trunk_pc, bottom_trunk_skeleton_pc, bottom_primary_branch_pc, 'entire', figname1, colors);
elseif strcmp(tree_id, 'tree4')
    roi = [-0.15, 0.2, -0.5, 0, 0.32, 0.7];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    roi = [-0.15, 0.2, -0.5, 0, 0.32, 0.7];
    bottom_trunk_index = findPointsInROI(refined_main_trunk_pc, roi);
    bottom_trunk_skeleton_pc = select(refined_main_trunk_pc, bottom_trunk_index);

    roi = [-0.15, 0.2, -0.5, 0, 0.32, 0.7];
    bottom_primary_branch_index = findPointsInROI(coarse_branch_pc, roi);
    bottom_primary_branch_pc = select(coarse_branch_pc, bottom_primary_branch_index);

    colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
    figname1 = 'Bottom primary branch 1';
    individual_branch_visualization(P, bottom_trunk_pc, bottom_trunk_skeleton_pc, bottom_primary_branch_pc, 'entire', figname1, colors);
end

%%
