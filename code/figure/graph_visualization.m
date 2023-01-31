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

file = files(3).name;
[filepath, tree_id, ext] = fileparts(file);
skel_filename_format = '_contract_*_skeleton.mat';
skel_filename = search_skeleton_file(tree_id, fullfile(skel_folder, exp_id), skel_filename_format);

skel_filepath = fullfile(skel_folder, exp_id, compare_exp_id, skel_filename);
load(skel_filepath, 'P'); % P results from skeleton operation
PP = P;

skel_filepath = fullfile(skel_folder, exp_id, skel_filename);
load(skel_filepath, 'P'); % P results from skeleton operation

% visualization purpose only and show the original point clouds
original_pt_normalized = P.original_pt;
original_pt_normalized_location = original_pt_normalized.Location;
desired_pt = 30000;
ratio = desired_pt / original_pt_normalized.Count;
pt = pcdownsample(original_pt_normalized, 'random', ratio); % visualization purpose only!

figure('Name', 'Tree');
ax1 = subplot(1, 3, 1);
% pcshow(pt, 'MarkerSize', 30); hold on
% set(gcf, 'color', 'none'); set(gca, 'color', 'none');
% plot_by_weight(P.spls, P.spls_density, false); hold on
scatter3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), 200, '.'); hold on
plot_connectivity(P.spls, P.spls_adj, 2, [1, 0, 0]);
grid on; axis equal;

ax2 = subplot(1, 3, 2);
% plot_by_weight(PP.spls, PP.spls_density, false); hold on
scatter3(PP.spls(:, 1), PP.spls(:, 2), PP.spls(:, 3), 200, '.'); hold on
plot_connectivity(PP.spls, PP.plain_mst_spls_adj, 2, [1, 0, 0]);
grid on; axis equal;

ax3 = subplot(1, 3, 3);
plot_by_weight(P.spls, P.spls_density, false); hold on
plot_connectivity(P.spls, P.mst_spls_adj, 2, [1, 0, 0]);
grid on; axis equal;

Link = linkprop([ax1, ax2, ax3], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);

%% branch segment
roi = [0.1, 0.5, -0.2, 0.1, 0.4, 0.7];
spls_pc = pointCloud(P.spls);
tmp_index = findPointsInROI(spls_pc, roi);

spls_zoom = P.spls(tmp_index, :);
spls_adj_zoom = P.spls_adj(tmp_index, tmp_index);
plain_mst_spls_adj_zoom = PP.plain_mst_spls_adj(tmp_index, tmp_index);
mst_spls_adj_zoom = P.mst_spls_adj(tmp_index, tmp_index);

figure('Name', 'Zoom-in branch');
ax1 = subplot(1, 3, 1);
scatter3(spls_zoom(:, 1), spls_zoom(:, 2), spls_zoom(:, 3), 200, '.'); hold on
plot_connectivity(spls_zoom, spls_adj_zoom, 2, [1, 0, 0]);
grid on; axis equal;

ax2 = subplot(1, 3, 2);
scatter3(spls_zoom(:, 1), spls_zoom(:, 2), spls_zoom(:, 3), 200, '.'); hold on
plot_connectivity(spls_zoom, plain_mst_spls_adj_zoom, 2, [1, 0, 0]);
grid on; axis equal;

ax3 = subplot(1, 3, 3);
plot_by_weight(P.spls, P.spls_density, false, tmp_index); hold on
plot_connectivity(spls_zoom, mst_spls_adj_zoom, 2, [1, 0, 0]);
grid on; axis equal;

Link = linkprop([ax1, ax2, ax3], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);

%% trunk segment
roi = [0, 0.2, -0.05, 0.05, -0.8, -0.4];
spls_pc = pointCloud(P.spls);
tmp_index = findPointsInROI(spls_pc, roi);

spls_zoom = P.spls(tmp_index, :);
spls_adj_zoom = P.spls_adj(tmp_index, tmp_index);
plain_mst_spls_adj_zoom = PP.plain_mst_spls_adj(tmp_index, tmp_index);
mst_spls_adj_zoom = P.mst_spls_adj(tmp_index, tmp_index);

figure('Name', 'Zoom-in branch');
ax1 = subplot(1, 3, 1);
scatter3(spls_zoom(:, 1), spls_zoom(:, 2), spls_zoom(:, 3), 200, '.'); hold on
plot_connectivity(spls_zoom, spls_adj_zoom, 2, [1, 0, 0]);
grid on; axis equal;

ax2 = subplot(1, 3, 2);
scatter3(spls_zoom(:, 1), spls_zoom(:, 2), spls_zoom(:, 3), 200, '.'); hold on
plot_connectivity(spls_zoom, plain_mst_spls_adj_zoom, 2, [1, 0, 0]);
grid on; axis equal;

ax3 = subplot(1, 3, 3);
plot_by_weight(P.spls, P.spls_density, false, tmp_index); hold on
plot_connectivity(spls_zoom, mst_spls_adj_zoom, 2, [1, 0, 0]);
grid on; axis equal;

Link = linkprop([ax1, ax2, ax3], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);

% plot_by_weight(PP.spls, PP.spls_density, true); hold on
% % plot3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), '.b', 'MarkerSize', 10); hold on
% plot_connectivity(PP.spls, PP.plain_mst_spls_adj, 2, [1, 0, 0]);
% grid on; axis equal;

% G = P.inverse_weighted_graph;
% LWidths = 5*G.Edges.Weight/max(G.Edges.Weight);
% figure;
% graph = plot(G,'LineWidth',LWidths);