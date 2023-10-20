clc; clear;  close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'E:\Data\FLIP\apple_KNX_03182022\exp_13M\Individual_Tree\Tree\Clean'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\apple_KNX_03182022\skeleton'; % folder storing extracted skeleton
exp_id = 'hc_downsample_iter_7';
extension = '.mat';

files = dir(fullfile(skel_folder, exp_id, ['tree*' extension]));
files = natsortfiles(files);

%%====================================%%
%%=====structure segmentation para=====%%
%%====================================%%
options.DEBUG = false;                                       % plot graph prior to MST
options.LOGGING = false;                                   % logging
options.TRUNK_REFINEMENT = true;          % trunk skeleton refinement
options.BRANCH_REFINEMENT = true;        % branch skeleton refinement
options.SAVE_PARAS = true;                             % save parameters for each tree
options.LOAD_PARAS = false;                            
options.SAVE_FIG = true;                                    % plot and save figures

%%====================================%%
%%=====architecture trait extraction para=====%%
%%====================================%%
options.SHOW = false;
options.SHOW_BRANCH = false;
options.SAVE = true;
options.CLEAR = false;
options.TO_FUSION = false;

target_trees = [2, 3, 14, 15, 16, 22, 23, 24];

for k = 1:length(target_trees)
    i = target_trees(k);
    file = files(i).name;
    [filepath, name, ext] = fileparts(file);
    split_file = split(name, '_');
    tree_id = split_file{1};
    disp(['=========Tree ' num2str(tree_id) ' ========='])
%     segmentation(data_folder, skel_folder, tree_id, exp_id, options);
    nt = branch_trait(skel_folder, tree_id, exp_id, '_branch_trait.xlsx', options);
    if k == 1
        T = nt;
    else
        T = vertcat(T, nt);
    end
end

output_folder = fullfile(skel_folder, '..', 'characterization', exp_id);
branch_excel_filepath = fullfile(output_folder, 'branch_trait1.xlsx');
writetable(T, branch_excel_filepath, 'Sheet', 'Branch_Level_Traits');
