clc; clear; 
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton\grid_downsample'; % folder storing original point cloud
extension = '.ply';
mat_extension = '.mat';

files = dir(fullfile(data_folder, ['tree*' mat_extension]));

pc_counts = [];

for i = 1:length(files)
    disp(['=========Tree ' num2str(i) ' ========='])
    file = files(i).name;
    [filepath, name, ext] = fileparts(file);

    if strcmp(ext, extension)
        pc = pcread(fullfile(data_folder, file));
        count = pc.Count;
    else
        load(fullfile(data_folder, file), 'P');
        count = P.npts;
    end

    pc_counts = [pc_counts, count];
end

pc_counts
disp(['Min: ', num2str(min(pc_counts), '%.2f')]);
disp(['Max: ', num2str(max(pc_counts), '%.2f')]);
disp(['Mean: ', num2str(mean(pc_counts), '%.2f')]);