clc; clear;  close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row16\skeleton'; % folder storing extracted skeleton
exp_id = 'hc_downsample_iter_7';
extension = '.mat';

files = dir(fullfile(skel_folder, exp_id, ['tree*' extension]));
files = natsortfiles(files);

%%====================================%%
%%=====architecture trait extraction para=====%%
%%====================================%%
options.DEBUG = false;                                       % plot graph prior to MST
options.LOGGING = false;                                   % logging
options.TRUNK_REFINEMENT = false;          % trunk skeleton refinement
options.BRANCH_REFINEMENT = false;        % branch skeleton refinement
options.SAVE_PARAS = false;                             % save parameters for each tree
options.LOAD_PARAS = false;                            
options.SAVE_FIG = true;                                    % plot and save figures
options.SHOW_CLUSTER_SPLIT = false;

%%====================================%%
%%=====architecture trait extraction para=====%%
%%====================================%%
options.SHOW = false;
options.SHOW_BRANCH = false;
options.SAVE = false;
options.CLEAR = false;
options.TO_FUSION = true;

for i = 1:length(files)
    file = files(i).name;
    [filepath, name, ext] = fileparts(file);
    split_file = split(name, '_');
    tree_id = split_file{1};
    disp(['=========Tree ' num2str(tree_id) ' ========='])
    segmentation(data_folder, skel_folder, tree_id, exp_id, options);
%     trait(name, skel_folder, exp_id, '_branch_test.xlsx', options);
end
