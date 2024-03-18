% This script created the visualization of tree meshes

% Define the directory path
directory = 'E:\Data\Fusion\FTB55\stl\row13';

% Find STL files containing "Full_Tree" in the directory
file_list = dir(fullfile(directory, '*Full_Tree*.stl'));

% Load and plot the first 5 trees
figure;
hold on;
for i = 1:min(9, numel(file_list))  % Load up to 7 trees
    % Load the STL file
    tree = stlread(fullfile(directory, file_list(i).name));
    
    % Apply Y-offset (1 meter) to each tree starting from the 2nd tree
    if i > 1
        tree = triangulation(tree.ConnectivityList, tree.Points + [0, (i - 0.8), 0]);  % Apply Y-offset
    end
    
    % Plot the tree
    trisurf(tree, 'FaceColor', 'black', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

% Turn off ticks
set(gca, 'xtick', [], 'ytick', [], 'ztick', []); axis off;

% Adjust view
view(3);
% maximize the figure window to fullscreen
set(gcf, 'WindowState', 'maximized');

% Turn off hold
hold off;
axis equal;

%% Set up the video writer
video_filepath = fullfile("C:\Users\tq42\OneDrive - Cornell University\Branch_Completion_2023", 'Fusion_Tree_Row13.avi');
writerObj = VideoWriter(video_filepath); % specify the file name and format
writerObj.FrameRate = 10; % set the frame rate (frames per second)
open(writerObj); % open the video writer

% capture each frame of the plot and write it to the video file
for t = 1:100 % Change the range as needed
    % rotate the plot for each frame (optional)
    view(3*t, 20);
    % capture the current frame
    frame = getframe(gcf);
    % write the frame to the video file
    writeVideo(writerObj, frame);
end
close(writerObj); % close the video writer
