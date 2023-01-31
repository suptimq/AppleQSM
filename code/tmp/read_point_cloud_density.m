clc; clear; 
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
extension = '.ply';

files = dir(fullfile(data_folder, ['tree*' extension]));

pc_counts = [];

for i = 1:length(files)-1
    disp(['=========Tree ' num2str(i) ' ========='])
    file = files(i).name;
    [filepath, name, ext] = fileparts(file);

    pc = pcread(fullfile(data_folder, file));

    pc_counts = [pc_counts, pc.Count];
end

pc_counts
disp(['Min: ', num2str(min(pc_counts), '%.2f')]);
disp(['Max: ', num2str(max(pc_counts), '%.2f')]);
disp(['Mean: ', num2str(mean(pc_counts), '%.2f')]);