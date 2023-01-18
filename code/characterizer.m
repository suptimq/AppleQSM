clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton'; % folder storing extracted skeleton
exp_id = 'multiplier_by_3_cpc_sphere_radius_002';
extension = '.ply';

files = dir(fullfile(data_folder, ['tree*' extension]));

%%====================================%%
%%=====architecture trait extraction para=====%%
%%====================================%%
options.DEBUG = false;                                       % plot graph prior to MST
options.LOGGING = false;                                   % logging
options.TRUNK_REFINEMENT = false;          % trunk skeleton refinement
options.BRANCH_REFINEMENT = true;        % branch skeleton refinement
options.SAVE_PARAS = false;                             % save parameters for each tree
options.LOAD_PARAS = false;                            
options.SAVE_FIG = false;                                    % plot and save figures
options.SHOW_CLUSTER_SPLIT = false;

%%====================================%%
%%=====architecture trait extraction para=====%%
%%====================================%%
options.SHOW = false;
options.SHOW_BRANCH = false;
options.SAVE = true;
options.CLEAR = false;
options.TO_FUSION = false;

for i = 1:length(files)
    disp(['=========Tree ' num2str(i) ' ========='])
    file = files(i).name;
    [filepath, name, ext] = fileparts(file);
%     segmentation(data_folder, skel_folder, name, exp_id, options);
    trait(name, skel_folder, exp_id, '_branch_test.xlsx', options);
end
