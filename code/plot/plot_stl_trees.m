% This script created the visualization of tree meshes

% Define the directory path
directory = 'E:\Data\Fusion\FTB55\stl\row15';

% Find STL files containing "Full_Tree" in the directory
file_list = dir(fullfile(directory, '*Full_Tree*.stl'));

% Load and plot the first 5 trees
figure;
hold on;
for i = 1:min(7, numel(file_list))  % Load up to 5 trees
    % Load the STL file
    tree = stlread(fullfile(directory, file_list(i).name));
    
    % Apply Y-offset (1 meter) to each tree starting from the 2nd tree
    if i > 1
        tree = triangulation(tree.ConnectivityList, tree.Points + [0, (i - 0.8), 0]);  % Apply Y-offset
    end
    
    % Plot the tree
    trisurf(tree, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

% Turn off ticks
set(gca, 'xtick', [], 'ytick', [], 'ztick', []); axis off;

% Adjust view
view(3);


% Turn off hold
hold off;
axis equal;

