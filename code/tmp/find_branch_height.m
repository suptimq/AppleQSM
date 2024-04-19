% This script computes branch heights for each tree branch and writes them
% to an Excel file. It also computes the offset of each tree branch based
% on the difference between the point clouds from PCD and PLY files,
% which is because of the inconsistent normalization....
%
% Paths to data folders and Excel file are defined first. Then, it reads
% the Excel file and initializes arrays to store branch heights and root
% file information. Next, it computes the offset for each tree branch by
% comparing the point clouds from PCD and PLY files.
%
% After computing the offset, it loops through each tree branch folder to
% compute the branch heights. For each root file, it extracts the
% tree index, section index, and color from the file name and loads the
% corresponding point cloud. It then computes the mean z-coordinate of the
% point cloud as the branch height and adjusts it by the computed offset.
%
% Finally, it updates the branch heights in the Excel file and writes the
% updated table back to the Excel file.
%
% Note: This script assumes that the structure of the data folders and the
% naming convention of the files are consistent.

% Define paths
data_folder = 'D:\Data\Apple_Orchard\Lailiang_Cheng\LLC_02022022\Row13_Raw';
primary_folder = 'D:\Data\Apple_Orchard\Lailiang_Cheng\LLC_02022022\Row13_Primary';
xlsx_filepath = 'D:\Data\Apple_Orchard\Lailiang_Cheng\Field_Measurements.xlsx';
sheet_name = 'Row13_Flat';

% Get list of folders with foldername as treeX_branch
branch_folders = dir(fullfile(data_folder, 'tree*_branch'));
num_branches = numel(branch_folders);

% Initialize cell array to store branch heights
branch_heights = cell(num_branches, 1);
% Initialize cell array to store root files
root_file_cell = cell(num_branches, 1);

% Read existing xlsx file
T = readtable(xlsx_filepath, 'Sheet', sheet_name);

%% Compute Offset
% Initialize cell array to store offsets for each tree
tree_offsets = cell(numel(branch_folders), 1);
% Initialize cell array to store trunk root z-coord for each tree
tree_roots = cell(numel(branch_folders), 1);

% Loop through each tree branch folder
for i = 1:numel(branch_folders)
    branch_folder = branch_folders(i).name;
    tree_idx = strtok(branch_folder, '_');
    tree_idx = tree_idx(5:end);
    
    % Find the first data entry for this tree in the Excel file
    tree_rows = T.Tree_Index == str2double(tree_idx);
    first_entry_idx = find(tree_rows, 1);
    
    % Form the filename
    section_idx = T.Section_Index(first_entry_idx);
    color = T.Color{first_entry_idx};
    filename = sprintf('Section%d_%s_Primary', section_idx, color);
    
    % Load point cloud files
    pcd_filepath = fullfile(primary_folder, branch_folder, [filename '.pcd']);
    ply_filepath = fullfile(primary_folder, branch_folder, [filename '.ply']);
    
    % Load point clouds
    ptCloud_pcd = pcread(pcd_filepath);
    ptCloud_ply = pcread(ply_filepath);
    
    % Compute offset
    offset = mean(ptCloud_pcd.Location - ptCloud_ply.Location);
    
    % Store offset for this tree branch
    tree_offsets{i} = offset;

    % Load trunk point cloud files
    pcd_filepath = fullfile(primary_folder, branch_folder, 'trunk.pcd');
    ptCloud_trunk = pcread(pcd_filepath);
    tree_roots{i} = min(ptCloud_trunk.Location(:, 3));

    % Adjust the coordinates of the root files
    branch_root_files = dir(fullfile(branch_folders(i).folder, branch_folder, '*_Root.ply'));
    for j = 1:numel(branch_root_files)
        branch_root_filename = branch_root_files(j).name;
        branch_root_filepath = fullfile(branch_root_files(j).folder, branch_root_filename);

        branch_root_pcd = pcread(branch_root_filepath);
        branch_root_location = branch_root_pcd.Location;
        branch_root_location = branch_root_location + offset;
        branch_root_offset_pcd = pointCloud(branch_root_location, 'Color', branch_root_pcd.Color);
        
        save_branch_root_path = fullfile(primary_folder, branch_folder, [branch_root_filename(1:end-4) '.pcd']);
        pcwrite(branch_root_offset_pcd, save_branch_root_path);
    end
end

%% Compute Branch Height
% Loop through each branch folder
for i = 1:num_branches
    tree_root = tree_roots{i};
    tree_offset = tree_offsets{i};
    branch_folder = branch_folders(i).name;
    branch_path = fullfile(data_folder, branch_folder);
    
    % Find all files ending with Root
    root_files = dir(fullfile(branch_path, '*Root.ply'));
    if isempty(root_files)
        error('No Root file found in folder %s', branch_folder);
    end
    root_file_cell{i} = root_files;

    % Initialize array to store heights of all roots in this branch
    heights_this_branch = zeros(numel(root_files), 1);
    
    % Loop through each Root file in the branch folder
    for j = 1:numel(root_files)
        % Extract tree_idx, section_idx, and color from root file name
        root_name = root_files(j).name;
        [tree_idx, remain] = strtok(root_name, '_');
        [section_idx, remain] = strtok(remain(2:end), '_');
        color = strtok(remain(2:end), '_');
        
        % Load the Root file
        root_filepath = fullfile(branch_path, root_name);
        ptCloud = pcread(root_filepath);
        
        % Compute mean value of point cloud as branch height
        branch_height = mean(ptCloud.Location(:,3)); % Assuming z-coordinate is height
        heights_this_branch(j) = branch_height + tree_offset(3) - tree_root;
    end
    
    % Store heights of all roots in this branch
    branch_heights{i} = heights_this_branch * 100;
end

%% Update Meta File
% Query Tree Index, Section Index, and Color to find the corresponding row
for i = 1:num_branches
    branch_folder = branch_folders(i).name;
    tree_idx = strtok(branch_folder, '_');
    tree_idx = tree_idx(5:end);
    root_files = root_file_cell{i};
    % Write branch heights to the xlsx file
    for j = 1:numel(branch_heights{i})
        % Extract tree_idx, section_idx, and color from root file name
        root_name = root_files(j).name;
        [section_idx, remain] = strtok(root_name, '_');
        section_idx = section_idx(8:end);
        [color, remain] = strtok(remain(2:end), '_');
        
        % Find corresponding row
        idx = find(T.Tree_Index == str2double(tree_idx) & T.Section_Index == str2double(section_idx) & strcmp(T.Color, color));
        if isempty(idx)
            error('Corresponding row not found for Tree Index %s, Section Index %s, Color %s', tree_idx, section_idx, color);
        end
        
        % Write branch height to the xlsx file
        T.Branch_Height(idx) = branch_heights{i}(j);
    end
end

% Write updated table to xlsx file
writetable(T, xlsx_filepath, 'Sheet', sheet_name);
disp('Branch heights written to Excel file.');
