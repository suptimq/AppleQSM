clc; clear;  close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);
path('config', path);

config = yaml.loadFile("config\iros_2024.yaml");

% load experimental parameters
mat_extension = config.experiment.mat_extension{1};
data_folder = config.experiment.data_folder{1};
models = config.experiment.models;
mode  = config.experiment.mode{1};
if yaml.isNull(config.experiment.exp_name)
    exp_name='.';
else
    exp_name = config.experiment.exp_name;
end
exp_folder = config.experiment.exp_folder{1};

result_folder = fullfile(data_folder, exp_folder);
if ~exist(result_folder, "dir")
    mkdir(result_folder);
end

%%====================================%%
%%=====structure segmentation para=====%%
%%====================================%%
options.SEG_PARA = config.segmentation;
options.DEBUG = config.segmentation.options.DEBUG;
options.LOGGING = config.segmentation.options.LOGGING;
options.TRUNK_REFINEMENT = config.segmentation.options.TRUNK_REFINEMENT;
options.BRANCH_REFINEMENT = config.segmentation.options.BRANCH_REFINEMENT;        
options.SAVE_PARAS = config.segmentation.options.SAVE_PARAS;
options.SAVE_FIG = config.segmentation.options.SAVE_FIG;
options.SEG_ON = config.segmentation.options.SEG_ON;

%%====================================%%
%%=====architecture trait extraction para=====%%
%%====================================%%
options.CHAR_PARA = config.characterization;
options.SHOW = config.characterization.options.SHOW;
options.SHOW_BRANCH = config.characterization.options.SHOW_BRANCH;
options.SAVE =config.characterization.options.SAVE;
options.CLEAR = config.characterization.options.CLEAR;
options.OUTPUT = config.characterization.options.OUTPUT;
options.CHAR_ON =config.characterization.options.CHAR_ON;

% create cell array for table data
data = cell(numel(models), config.experiment.num_tree+1);

for i = 1:length(models)
    model = models{i}{1};
    tree_folder = fullfile(data_folder, model, mode);
    skel_folder = fullfile(data_folder, model, mode, exp_folder, 'Skeleton');

    % load skeleton files
    mat_files = dir(fullfile(skel_folder, exp_name, ['tree*' mat_extension]));
    mat_files = natsortfiles(mat_files);

    data{i, 1} = model;
    if options.SEG_ON
        for k = 1:length(files)
            file = mat_files(k).name;
            [filepath, name, ext] = fileparts(file);
            split_file = split(name, '_');
            tree_id = split_file{1};
            disp(['=========Tree ' num2str(tree_id) ' ========='])
            num_primary_branch = segmentation(data_folder, skel_folder, tree_id, exp_name, options);
            data{i, 1+k} = num_primary_branch;
        end
    
        %% Save the table to a CSV file
        % Create table
        T = cell2table(data, 'VariableNames', {'model_name', 'tree1', 'tree2', 'tree3', 'tree4', 'tree5', 'tree6', 'tree7', 'tree8', 'tree9'});    

        csv_filepath_output = fullfile(result_folder, 'Branch_Quantity.csv');  % Define filename
        % Check if the file exists
        if exist(csv_filepath_output, 'file')
            % If the file exists, append the data
            writetable(T, csv_filepath_output, 'WriteMode', 'append');
            disp(['Branch quantity appended to: ' csv_filepath_output]);
        else
            % If the file does not exist, write the data to a new file
            writetable(T, csv_filepath_output);  % Write table to CSV file
            disp(['Branch quantity saved to: ' csv_filepath_output]);
        end

    end
    
    if options.CHAR_ON
        for k = 1:length(mat_files)
            file = mat_files(k).name;
            [filepath, name, ext] = fileparts(file);
            split_file = split(name, '_');
            tree_id = split_file{1};
            disp(['=========Tree ' num2str(tree_id) ' ========='])
            nt = branch_trait(skel_folder, tree_id, exp_name, '_branch_trait.xlsx', options);
            if k == 1
                T = nt;
            else
                T = vertcat(T, nt);
            end
        end
        
        output_folder = fullfile(skel_folder, '..', 'Characterization', exp_name);
        branch_csv_filepath = fullfile(output_folder, 'Branch_Trait.csv');
        writetable(T, branch_csv_filepath);
    end

end
