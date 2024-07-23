% Define the folder containing PCD files
% folder_path = 'E:\Result\LLC_02022022\Row13\Generator2-AdaPoinTr-Skeleton-GAN_FTB55-v2_CDL1_SkelLoss-Coordinate-Only-Supervised-0.01_Repulsion_CPC-2nd-Stage_Finetune\Primary\tree1_segmented'; % 
folder_path = 'D:\Data\Apple_Orchard\Lailiang_Cheng\LLC_02022022\Row13_Primary\tree1_branch';
output_folder = 'C:\Users\tq42\OneDrive - Cornell University\Branch_Completion_2023\Tree1\Raw_Branch';

% Get a list of all PCD files in the folder
pcd_files = dir(fullfile(folder_path, '*.pcd'));

% Remove 'rest_branch.pcd' and 'trunk.pcd' from the list
exclude_files = {'rest_branch.pcd', 'trunk.pcd'};
pcd_files = pcd_files(~ismember({pcd_files.name}, exclude_files));

% Define border size
border_size = 100; % Adjust the border size as needed

% Loop through each PCD file
for i = 1:numel(pcd_files)
    % Read PCD file
    [filepath, name, ext] = fileparts(pcd_files(i).name);
    file_path = fullfile(folder_path, pcd_files(i).name);
    pcd_data = pcread(file_path);
    
    % Create a new point cloud with black color
    if isempty(pcd_data.Color)
        num_points = size(pcd_data.Location, 1);
        black_color = repmat([0, 0, 0], num_points, 1);
        pcd_with_black_color = pointCloud(pcd_data.Location, 'Color', black_color);
    else
        pcd_with_black_color = pcd_data;
    end

    % Plot and save PNG image
    fig = figure('Visible', 'off'); % Create figure without displaying it
    pcshow(pcd_with_black_color);
    set(gcf, 'color', 'white'); set(gca, 'color', 'white');
    axis equal; axis off;
% 
%     % Add black border
%     frame = getframe(gca);
%     im = frame2im(frame);
%     im_with_border = padarray(im, [border_size, border_size], 0, 'both');

    view([90, 0]);
    filename = fullfile(output_folder, [name, '.png']);
    saveas(fig, filename); % Save figure as PNG image

    % Create video writer object
    video_filepath = fullfile(output_folder, [name '_map.avi']);
    writerObj = VideoWriter(video_filepath); % Specify the file name and format
    writerObj.FrameRate = 10; % Set the frame rate (frames per second)
    open(writerObj); % Open the video writer
    
    % Capture each frame of the plot and write it to the video file
    for t = 1:100 % Change the range as needed
        % Rotate the plot for each frame (optional)
        view(3*t, 20);
        % Capture the current frame
        frame = getframe(gcf);
        % Write the frame to the video file
        writeVideo(writerObj, frame);
    end
    close(writerObj); % Close the video writer

    close(fig); % Close the figure
end
