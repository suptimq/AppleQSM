clc; clear;  close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

extension = '.mat';
data_folder = 'E:\Result\LLC_02022022\Row13';
models = {'Raw_Incomplete_Trees';
                     'AdaPoinTr_FTB55-v2_CDL1_Finetune';
                     'AdaPoinTr_LTB81-v4_CDL1_Finetune';
                     'Generator2-AdaPoinTr-Skeleton-GAN_FTB55-v2_CDL1_SkelLoss-Supervised-0.01_Finetune'
                     'Generator2-AdaPoinTr-Skeleton-GAN_LTB81-v4_CDL1_SkelLoss-Supervised-0.01_Finetune'};
mode  = 'Primary';
exp_id = 'hc_downsample_iter_7';

result_folder = fullfile(data_folder, 'AppleQSM');
if ~exist(result_folder, "dir")
    mkdir(result_folder);
end

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

% Create cell array for table data
data = cell(numel(models), 10);

for i = 1:length(models)
    model = models{i};
    tree_folder = fullfile(data_folder, model, mode);
    skel_folder = fullfile(data_folder, model, mode, 'AppleQSM', 'Skeleton');

    files = dir(fullfile(skel_folder, exp_id, ['tree*' extension]));
    files = natsortfiles(files);

    data{i, 1} = model;
    if options.SEG_ON
        for k = 1:length(files)
            file = files(k).name;
            [filepath, name, ext] = fileparts(file);
            split_file = split(name, '_');
            tree_id = split_file{1};
            disp(['=========Tree ' num2str(tree_id) ' ========='])
            num_primary_branch = segmentation(data_folder, skel_folder, tree_id, exp_id, options);
            data{i, 1+k} = num_primary_branch;
        end

        % Create table
        T = cell2table(data, 'VariableNames', {'model_name', 'tree1', 'tree2', 'tree3', 'tree4', 'tree5', 'tree6', 'tree7', 'tree8', 'tree9'});
        % Save table to a CSV file
        filename = fullfile(result_folder, 'Branch_Quantity.csv');  % Define filename
%         writetable(T, filename);  % Write table to CSV file
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
        branch_csv_filepath = fullfile(output_folder, 'Branch_Trait.csv');
        writetable(T, branch_csv_filepath);
    end

end
