% This script re-normalized the pcds to match the normalized coordinates between
% the 1st round manual segmented branches (for PP) and the 2nd round manual segmented branches (for IROS)

tree_folder = 'E:\Result\LLC_02022022\Row13\Full_Branch_Segmentaion_Tmp\Primary\1st_Pruned';

pcd_files = dir(fullfile(tree_folder, '*.pcd'));
pcd_files = natsortfiles(pcd_files);

branch_folder = 'E:\Result\LLC_02022022\Row13\Full_Branch_Segmentaion_Tmp\Primary\AppleQSM\Segmentation';
trunk_folder = 'D:\Data\Apple_Orchard\Lailiang_Cheng\LLC_02022022\LLC_02022022_Row13';

% loop each tree
for i = 1
    pcd_filename = pcd_files(i).name;
    [filepath, name, ext] = fileparts(pcd_filename);
    pcd_filepath = fullfile(tree_folder, pcd_filename);

    % compute mean
    pcd = pcread(pcd_filepath);
    pcd_location = pcd.Location;
    pcd_location_mean = mean(pcd_location, 1);

    tree_branch_folder = fullfile(branch_folder, [name '_segmented']);
    tree_trunk_folder = fullfile(trunk_folder, [name '_branch']);
    ply_filepath = fullfile(tree_trunk_folder, 'Trunk.ply');
    % load and normalize
    ply = pcread(ply_filepath);
    ply_location = ply.Location;
    ply_location_normalized = ply_location - pcd_location_mean;

    saved_filepath = fullfile(tree_branch_folder, 'trunk.pcd');
    pcwrite(pointCloud(ply_location_normalized, 'Color', ply.Color), saved_filepath);

%     %% loop all ply files (previously manually cropped)
%     tree_branch_folder = fullfile(branch_folder, [name '_segmented']);
%     ply_files = dir(fullfile(tree_branch_folder, '*.ply'));
%     for j = 1:length(ply_files)
%         ply_filename = ply_files(j).name;
%         [~, ply_name, ~] = fileparts(ply_filename);
%         ply_filepath = fullfile(tree_branch_folder, ply_filename);
%         
%         % load and normalize
%         ply = pcread(ply_filepath);
%         ply_location = ply.Location;
%         ply_location_normalized = ply_location - pcd_location_mean;
% 
%         saved_filepath = fullfile(tree_branch_folder, [ply_name '.pcd']);
%         pcwrite(pointCloud(ply_location_normalized, 'Color', ply.Color), saved_filepath);
%     end
end
