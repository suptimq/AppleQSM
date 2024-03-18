% Define the folder containing the LAS file
folder_path = 'F:\FARO\Processed\Lailiang_Cheng\LLC_04022022';
pc_filepath = fullfile(folder_path, 'downsampled-100x_multi_rows.pcd');

% Load the LAS file
if endsWith(pc_filepath, '.las')
    las_data = lasFileReader(pc_filepath);
    ptCloud = readPointCloud(las_data);
    % Downsample the point cloud
    downsampled_pc = pcdownsample(ptCloud, 'random', 0.01); % Adjust the percentage as needed
else
    downsampled_pc = pcread(pc_filepath);
end

% Initialize figure
figure;
axis equal;

pcshow(downsampled_pc);
set(gcf, 'color', 'white'); set(gca, 'color', 'white');

%% Set up the video writer
video_filepath = fullfile("C:\Users\tq42\OneDrive - Cornell University\Branch_Completion_2023", 'Real_Orchard_Video.avi');
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
