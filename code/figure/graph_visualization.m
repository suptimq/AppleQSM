clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton'; % folder storing extracted skeleton
output_folder = 'C:\Users\tq42\Documents\ASABE_2022_Journal';
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

filename = fullfile(output_folder, 'graph_visualization_1');
figure;
ax1 = subplot(1, 2, 1);
% pcshow(pt, 'MarkerSize', 30); hold on
% set(gcf, 'color', 'none'); set(gca, 'color', 'none');
plot_by_weight(P.spls, P.spls_density, false); hold on
plot_connectivity(P.spls, P.spls_adj, 2, [1, .0, .0]);
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax2 = subplot(1, 2, 2);
plot_by_weight(P.spls, P.spls_density, false); hold on
plot_connectivity(P.spls, P.mst_spls_adj, 2, [1, .0, .0]);
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);
% view(-60, 10);
% print('-painters', '-dpdf', '-fillpage', '-r300', filename);

G = P.inverse_weighted_graph;
LWidths = 5*G.Edges.Weight/max(G.Edges.Weight);
figure;
graph = plot(G,'LineWidth',LWidths);