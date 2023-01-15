clc; clear; close all;
path('..', path)

data_folder = 'D:\skeletonization-master\cloudcontr_2_0\data\Row13_Raw'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton'; % folder storing extracted skeleton

tree_id = 'tree1';
pt_filename = [tree_id '.ply'];
skel_filename_format = '_contract_*_skeleton.mat';
skel_filename = search_skeleton_file(tree_id, skel_folder, skel_filename_format);

skel_filepath = fullfile(skel_folder, skel_filename);
load(skel_filepath, 'P'); % P results from skeleton operation

index = P.pts_label == 0;
color = repmat([255, 0, 0], length(P.pts_label), 1);
color(index, :) = repmat([0, 0, 255], sum(index(:)==1), 1);
pre_segmented_pc = pointCloud(P.pts, 'Color', color);

figure(1);
pcshow(pre_segmented_pc, 'MarkerSize', 30);
