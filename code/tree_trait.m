close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\apple_KNX_03182022\segmentation'; % folder storing extracted skeleton
exp_id = 'hc_downsample_iter_7';
extension = '.mat';

output_folder = fullfile(skel_folder, '..', 'characterization', exp_id);
files = dir(fullfile(skel_folder, exp_id, ['tree*' extension]));
files = natsortfiles(files);
skel_filename_format = '_contract_*_skeleton.mat';

%% create folder to save results
if ~exist(output_folder, 'dir')
    mkdir(output_folder)
end

T = table();
excel_filename = 'tree_trait.xlsx';
total_branch_recall = 0;
for i = 1:length(files)
    file = files(i).name;
    [filepath, name, ext] = fileparts(file);
    split_file = split(name, '_');
    tree_id = split_file{1};
    disp(['=========Tree ' tree_id ' ========='])
    exp_folder = fullfile(skel_folder, exp_id);
    skel_filename = search_skeleton_file(tree_id, exp_folder, skel_filename_format);
    skel_filepath = fullfile(exp_folder, skel_filename);
    load(skel_filepath, 'P');

    tree_height = P.main_trunk_height;
    trunk_length =  P.main_trunk_length;
    trunk_radius = P.trunk_radius;
    branch_recall = P.branch_counter;
    disp(['Branch recall: ', num2str(branch_recall)]);
    
    total_branch_recall = total_branch_recall + branch_recall;

    T = [T; {tree_id, num2str(tree_height, '%.3f'), num2str(trunk_length, '%.3f'), num2str(trunk_radius, '%.3f'), num2str(branch_recall, '%.3f')}];
end

disp(['Total branch recall: ', num2str(total_branch_recall)]);

T.Properties.VariableNames = {'Filename', 'Tree Height', 'Trunk Length', 'Trunk Radius', 'Branch Recall'};
excel_filepath = fullfile(output_folder, excel_filename);
writetable(T, excel_filepath, 'Sheet', 'Tree_Level_Traits_1');