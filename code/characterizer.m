clc; clear;  close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13\segmentation'; % folder storing extracted skeleton
exp_id = 'hc_downsample_iter_7\s1';
extension = '.mat';

files = dir(fullfile(skel_folder, exp_id, ['tree*' extension]));
files = natsortfiles(files);

%%====================================%%
%%=====structure segmentation para=====%%
%%====================================%%
options.DEBUG = false;                                       % plot graph prior to MST
options.LOGGING = false;                                   % logging
options.TRUNK_REFINEMENT = false;          % trunk skeleton refinement
options.BRANCH_REFINEMENT = false;        % branch skeleton refinement
options.SAVE_PARAS = true;                             % save parameters for each tree
options.LOAD_PARAS = false;                            
options.SAVE_FIG = true;                                    % plot and save figures
options.SEG_ON = false;

%%====================================%%
%%=====architecture trait extraction para=====%%
%%====================================%%
options.SHOW = false;
options.SHOW_BRANCH = false;
options.SAVE = true;
options.CLEAR = false;
options.TO_FUSION = false;
options.CHAR_ON = true;

if options.SEG_ON
    for k = 1:length(files)
        file = files(k).name;
        [filepath, name, ext] = fileparts(file);
        split_file = split(name, '_');
        tree_id = split_file{1};
        disp(['=========Tree ' num2str(tree_id) ' ========='])
        segmentation(data_folder, skel_folder, tree_id, exp_id, options);
    end
end

if options.CHAR_ON
    for k = 1:length(files)
        file = files(k).name;
        [filepath, name, ext] = fileparts(file);
        split_file = split(name, '_');
        tree_id = split_file{1};
        disp(['=========Tree ' num2str(tree_id) ' ========='])
        nt = branch_trait(skel_folder, tree_id, exp_id, '_branch_trait.xlsx', options);
        if k == 1
            T = nt;
        else
            T = vertcat(T, nt);
        end
    end
    
    output_folder = fullfile(skel_folder, '..', 'Characterization', exp_id);
    branch_excel_filepath = fullfile(output_folder, 'branch_trait_cs2.xlsx');
    writetable(T, branch_excel_filepath, 'Sheet', 'Branch_Level_Traits');
end
