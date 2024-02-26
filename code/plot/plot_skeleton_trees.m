% This script created the visualization of tree skeletons
% Several skeletons are not aligned well with the tree pcd (Need to figure the reason)

% Define the folder containing PCD files
folder_path = 'E:\Result\LLC_02022022\Row13\Raw_Incomplete_Trees\Primary';
skel_folder_path = fullfile(folder_path, "AppleQSM/Skeleton");

% Get a list of all PCD files in the folder
pcd_files = dir(fullfile(folder_path, '*.pcd'));
mat_files = dir(fullfile(skel_folder_path, '*.mat'));
num_files = numel(pcd_files);

% Initialize figure
figure;
hold on;

selected_trees = [1, 2,3, 4, 5, 6, 7, 8,9];
indices = [2,4,5,6,7];

% Loop through PCD files starting from the second one
for i = 1:length(selected_trees)
    if ismember(i, indices)
        z_offset = 0.05;
    else
        z_offset = 0;
    end
    if i == 3 || i == 9
        z_offset = 0.1;
    end
    % Read PCD file
    file_path = fullfile(folder_path, pcd_files(selected_trees(i)).name);
    pcd_data = pcread(file_path);
    skel_file_path = fullfile(skel_folder_path, mat_files(selected_trees(i)).name);
    load(skel_file_path, 'P');
    
    % Extract location coordinates
    location = pcd_data.Location;
    % Add 1 meter offset to Y coordinates
    location(:, 2) = location(:, 2) + 1*(i-1); % Assuming Y is the vertical axis
    % Plot the PCD data
    pcshow(pointCloud(location, 'Color', pcd_data.Color), 'MarkerSize', 15);
    set(gcf, 'color', 'white'); set(gca, 'color', 'white');
    plot3(P.spls(:, 1), P.spls(:, 2) + 1*(i-1), P.spls(:, 3)-z_offset, '.', 'MarkerSize', 20, 'Color', [1 0 0 0.3]);
end

% Turn off ticks
set(gca, 'xtick', [], 'ytick', [], 'ztick', []); axis off;
% maximize the figure window to fullscreen
set(gcf, 'WindowState', 'maximized');
% Adjust view
% upfront  view
view([90, 0])
saveas(gcf, fullfile(folder_path, 'skel_upfront.png'));
% top view
view([90, 90])
saveas(gcf, fullfile(folder_path, 'skel_top.png'));

% Turn off hold
hold off;
axis equal;
