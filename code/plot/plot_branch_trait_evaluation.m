extension = '.mat';
data_folder = 'E:\Result\LLC_02022022\Row13';
% models = {'Raw_Incomplete_Trees';
%                      'AdaPoinTr_FTB55-v2_CDL1_Finetune';
%                      'AdaPoinTr_LTB81-v4_CDL1_Finetune';
%                      'Generator2-AdaPoinTr-Skeleton-GAN_FTB55-v2_CDL1_SkelLoss-Supervised-0.01_Finetune';
%                      'Generator2-AdaPoinTr-Skeleton-GAN_LTB81-v4_CDL1_SkelLoss-Supervised-0.01_Finetune';
%                      'Generator2-AdaPoinTr-Skeleton-GAN_LTB81-v4_CDL1_SkelLoss-Supervised-0.01_Repulsion_CPC-2nd-Stage_Finetune'};

models = { 'AdaPoinTr_FTB55-v2_CDL1_Finetune';
                    'Generator2-AdaPoinTr-Skeleton-GAN_FTB55-v2_CDL1_SkelLoss-Supervised-0.01_Finetune';};

mode  = 'Primary';
intercept = false;

result_folder = fullfile(data_folder, 'AppleQSM2');

% Initialize arrays to store R-squared values
model_names = {};
diameter_linear_r2 = [];
diameter_robust_r2 = [];
angle_linear_r2 = [];
angle_robust_r2 = [];

% Loop through each model
for i = 1:length(models)
    model = models{i}; % Get the current model name
    
    % Construct the file path to the CSV file containing the data for the current model
    csv_filepath = fullfile(data_folder, model, mode, 'AppleQSM/Characterization/hc_downsample_iter_7/3DGAC_Matched_Branch_Trait.csv');
    
    % Read the CSV file into a table
    T = readtable(csv_filepath);
    
    % Define column indices for diameter and angle data
    diameter_col_index = [10, 14];
    angle_col_index = [9, 13];
    
    % Extract diameter data from the table
    diameter_data = T(:, diameter_col_index);
    
    % Remove rows with NaN values in both diameter columns
    ind = ~isnan(diameter_data{:, 1}) & ~isnan(diameter_data{:, 2});
    diameter_data = diameter_data(ind, :);
    
    % Extract angle data from the table
    angle_data = T(:, angle_col_index);
    
    % Remove rows with NaN values in both angle columns
    ind = ~isnan(angle_data{:, 1}) & ~isnan(angle_data{:, 2});
    angle_data = angle_data(ind, :);

    % Fit linear models to the diameter data with and without robust fitting
    mdlr_diameter = fitlm(diameter_data, 'Intercept', intercept); % Linear model
    mdblr_diameter = fitlm(diameter_data, 'Intercept', intercept, 'RobustOpts', 'huber'); % Linear model with robust fitting
    
    % Fit linear models to the angle data with and without robust fitting
    mdlr_angle = fitlm(angle_data, 'Intercept', intercept); % Linear model
    mdblr_angle = fitlm(angle_data, 'Intercept', intercept, 'RobustOpts', 'huber'); % Linear model with robust fitting
    
    % Store R-squared values for the current model
    model_names{end+1} = model;
    diameter_linear_r2(end+1) = mdlr_diameter.Rsquared.Ordinary;
    diameter_robust_r2(end+1) = mdblr_diameter.Rsquared.Ordinary;
    angle_linear_r2(end+1) = mdlr_angle.Rsquared.Ordinary;
    angle_robust_r2(end+1) = mdblr_angle.Rsquared.Ordinary;
end

% Create a table with R-squared values
R2_table = table(model_names', diameter_linear_r2', diameter_robust_r2', angle_linear_r2', angle_robust_r2', ...
    'VariableNames', {'model_name', 'diameter_linear_R2', 'diameter_robust_R2', 'angle_linear_R2', 'angle_robust_R2'});

%% Save the table to a CSV file
csv_filepath_output = fullfile(result_folder, 'R2_values.csv');
% Check if the file exists
if exist(csv_filepath_output, 'file')
    % If the file exists, append the data
    writetable(R2_table, csv_filepath_output, 'WriteMode', 'append');
    disp(['R-squared values appended to: ' csv_filepath_output]);
else
    % If the file does not exist, write the data to a new file
    writetable(R2_table, csv_filepath_output);
    disp(['R-squared values saved to: ' csv_filepath_output]);
end