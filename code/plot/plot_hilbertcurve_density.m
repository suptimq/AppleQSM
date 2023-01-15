clc; clear; close all;
path('..', path)
path('hilbertcurve_and_density_calculate', path)

data_folder = 'D:\skeletonization-master\cloudcontr_2_0\data\Row13_Raw'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton'; % folder storing extracted skeleton

tree_id = 'tree1';
pt_filename = [tree_id '.ply'];
skel_filename_format = '_contract_*_skeleton.mat';
skel_filename = search_skeleton_file(tree_id, skel_folder, skel_filename_format);

skel_filepath = fullfile(skel_folder, skel_filename);
load(skel_filepath, 'P'); % P results from skeleton operation

[~, index] = ismember(P.pts, P.original_pt.Location, 'row');
index = index(index ~= 0);

pc1 = before_density(P.pts_coordinate2distance, P.bin_size, P.num_iteration, P.original_pt);
location_ = pc1.Location;
intensity = pc1.Intensity;

figure(1);
plot_by_weight(location_(index, :), intensity(index))