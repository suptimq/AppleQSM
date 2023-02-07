%% setting
clear;clc;close all;

extension = '.pcd';
data_folder = 'D:\Data\Apple_Orchard\Lailiang_Cheng\LLC_04022022\Xiangtao_Segment\row 16'; % folder storing original point cloud
save_folder = fullfile(data_folder, '.'); % folder storing extracted skeleton

files = dir([data_folder '\' '*' extension]);
files = natsortfiles(files);

if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end

for i = 1:length(files)
    file = files(i);
    [filepath, name, ext] = fileparts(file.name);
    pc = pcread(fullfile(file.folder, file.name));
    pcwrite(pc, fullfile(save_folder, [name '.ply']), PLYFormat="binary");
end