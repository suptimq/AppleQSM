% This script created the visualization of tree pcds

% Define the folder containing PCD files
folder_path = 'E:\Result\LLC_02022022\Row13\Raw_Incomplete_Trees\Primary';

% Get a list of all PCD files in the folder
pcd_files = dir(fullfile(folder_path, '*.pcd'));
num_files = numel(pcd_files);

% Check if there are enough files
if num_files < 2
    error('There are not enough PCD files in the folder.');
end

% Initialize figure
figure;
hold on;

% Loop through PCD files starting from the second one
for i = 1:num_files
    % Read PCD file
    file_path = fullfile(folder_path, pcd_files(i).name);
    pcd_data = pcread(file_path);
    
    % Extract location coordinates
    location = pcd_data.Location;
    % Add 1 meter offset to Y coordinates
    location(:, 2) = location(:, 2) + 1*(i-1); % Assuming Y is the vertical axis
    % Plot the PCD data
    pcshow(pointCloud(location, 'Color', pcd_data.Color), 'MarkerSize', 30);
    set(gcf, 'color', 'white'); set(gca, 'color', 'white');
end

% Turn off ticks
set(gca, 'xtick', [], 'ytick', [], 'ztick', []); axis off;

% Adjust view
% upfront  view
view([90, 0])
saveas(gcf, fullfile(folder_path, 'upfront.png'));
% top view
view([90, 90])
saveas(gcf, fullfile(folder_path, 'top.png'));

% Turn off hold
hold off;
axis equal;
