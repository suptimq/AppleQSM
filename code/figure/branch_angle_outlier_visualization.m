clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13\segmentation'; % folder storing extracted skeleton
exp_id = 'hc_downsample_iter_7\s1';
extension = '.ply';

files = dir(fullfile(data_folder, ['tree*' extension]));

file = files(1).name;
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

if strcmp(tree_id, 'tree1')
    roi = [-0.05, 0.01, 0, 0.16, -0.18, -0.02];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    figure;
    pcshow(bottom_trunk_pc, 'MarkerSize', 40)
    set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');

elseif strcmp(tree_id, 'tree6')
    roi = [-0.05, 0.2, 0, 0.1, 0.18, 0.25];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    figure;
    pcshow(bottom_trunk_pc, 'MarkerSize', 40)
    set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');

elseif strcmp(tree_id, 'tree9')
    roi = [0, 0.1, 0, 0.1, -0.4, -0.28];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    figure;
    pcshow(bottom_trunk_pc, 'MarkerSize', 40)
    set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');

elseif strcmp(tree_id, 'tree5')
    roi = [-0.1, 0.02, 0, 0.06, 0.14, 0.22];
    bottom_trunk_index = findPointsInROI(original_pt_normalized, roi);
    bottom_trunk_pc = select(original_pt_normalized, bottom_trunk_index);

    figure;
    pcshow(bottom_trunk_pc, 'MarkerSize', 40)
    set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');


end