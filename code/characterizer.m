clc; clear;  close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);
path('config', path);

config_filepath = "config\iros_2024.yaml";
config = yaml.loadFile(config_filepath);

%% Skeletonization
% load downsampling parameters
options.bin_size = config.skeleton.bin_size;
options.num_iteration = config.skeleton.num_iteration;
options.downsample_num = config.skeleton.downsample_num;
options.downsample_mode = config.skeleton.downsample_mode; 
options.density_mode = config.skeleton.density_mode; 
options.gridStep = config.skeleton.gridStep;
options.USING_POINT_RING = GS.USING_POINT_RING;
options.sample_radius_factor = config.skeleton.sample_radius_factor;
options.SKEL_ON = config.skeleton.SKEL_ON;

% load experimental parameters
pcd_extension = config.experiment.pcd_extension{1};
data_folder = config.experiment.data_folder{1};
models = config.experiment.models;
mode  = config.experiment.mode{1};
if yaml.isNull(config.experiment.exp_name)
    exp_name='.';
else
    exp_name = config.experiment.exp_name;
end
exp_folder = config.experiment.exp_folder{1};

%% loop models
if options.SKEL_ON
    for k = 1:length(models)
        model = models{k}{1};
        tree_folder = fullfile(data_folder, model, mode);
        skel_folder = fullfile(data_folder, model, mode, exp_folder, 'Skeleton', 'Characterization', exp_name);
    
        % copy config to skeleton folder
        if ~exist(skel_folder, "dir")
            mkdir(skel_folder);
        end
        copyfile(config_filepath, skel_folder);
    
        pcd_files = dir(fullfile(tree_folder, ['*' pcd_extension]));
        pcd_files = natsortfiles(pcd_files);
        
        % loop pcd_files
        for i = 8
            filename = pcd_files(i).name;
            disp(['=========Tree ' num2str(filename) ' ========='])
            laplacian_skeleton(tree_folder, skel_folder, exp_name, filename, options);
        end
    end
end

%% Segmentation
% load experimental parameters
mat_extension = config.experiment.mat_extension{1};

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
options.SEGMENTATION = config.segmentation.options.SEGMENTATION;
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
        segmentation_folder = fullfile(data_folder, model, mode, exp_folder, 'Segmentation', 'Characterization', exp_name);
        % copy config to segmentation folder
        if ~exist(segmentation_folder, "dir")
            mkdir(segmentation_folder);
        end
        copyfile(config_filepath, segmentation_folder);
        for k = 9
            file = mat_files(k).name;
            [filepath, name, ext] = fileparts(file);
            split_file = split(name, '_');
            tree_id = split_file{1};
            disp(['=========Tree ' num2str(tree_id) ' ========='])
            num_primary_branch = segmentation(skel_folder, tree_id, exp_name, options);
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

        characterization_folder = fullfile(data_folder, model, mode, exp_folder, 'Characterization', exp_name);
        % copy config to characterization folder
        if ~exist(characterization_folder, "dir")
            mkdir(characterization_folder);
        end
        copyfile(config_filepath, characterization_folder);

        matched_qsm_filepath = fullfile(characterization_folder, '3DGAC_Matched_Branch_Trait.csv');
        if exist(matched_qsm_filepath, 'file')
            matched_qsm_table = readtable(matched_qsm_filepath);
        else
            matched_qsm_table = table;
        end

        for k = 1:length(mat_files)
            file = mat_files(k).name;
            [filepath, name, ext] = fileparts(file);
            split_file = split(name, '_');
            tree_id = split_file{1};
            disp(['=========Tree ' num2str(tree_id) ' ========='])
            nt = branch_trait(skel_folder, tree_id, exp_name, '_branch_trait.xlsx', matched_qsm_table, options);
            if k == 1
                T = nt;
            else
                T = vertcat(T, nt);
            end
        end

        if options.SHOW
            % maximize the figure window to fullscreen
            set(gcf, 'WindowState', 'maximized');
            % upfront  view
            view([90, 0])
            saveas(gcf, fullfile(characterization_folder, 'upfront.png'));
            % top view
            view([90, 90])
            saveas(gcf, fullfile(characterization_folder, 'top.png'));

            %% Set up the video writer
            video_filepath = fullfile(characterization_folder, 'pruning_indication_map.avi');
            writerObj = VideoWriter(video_filepath); % Specify the file name and format
            writerObj.FrameRate = 10; % Set the frame rate (frames per second)
            
            open(writerObj); % Open the video writer
            
            % Capture each frame of the plot and write it to the video file
            for t = 1:100 % Change the range as needed
                % Rotate the plot for each frame (optional)
                view(3*t, 20);
                % Capture the current frame
                frame = getframe(gcf);
                % Write the frame to the video file
                writeVideo(writerObj, frame);
            end
            close(writerObj); % Close the video writer
        end

        if options.SAVE
            branch_csv_filepath = fullfile(characterization_folder, 'Branch_Trait.csv');
            writetable(T, branch_csv_filepath);
        end

        close all;
    end

end
