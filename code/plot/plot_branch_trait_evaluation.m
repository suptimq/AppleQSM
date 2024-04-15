config = yaml.loadFile("config\default.yaml");

extension = config.experiment.mat_extension{1};
data_folder = config.experiment.data_folder{1};
models = config.experiment.models;
mode  = config.experiment.mode{1};
if yaml.isNull(config.experiment.exp_name)
    exp_name='.';
else
    exp_name = config.experiment.exp_name;
end
exp_folder = config.experiment.exp_folder{1};
intercept = false;

result_folder = fullfile(data_folder, exp_folder);

% Initialize arrays to store R-squared values
model_names = {};
diameter_linear_r2 = [];
diameter_robust_r2 = [];
angle_linear_r2 = [];
angle_robust_r2 = [];

% Initialize arrays to store MSE and RMSE values
diameter_mse = zeros(length(models), 1);
angle_mse = zeros(length(models), 1);
diameter_rmse = zeros(length(models), 1);
angle_rmse = zeros(length(models), 1);

% Define column indices for diameter and angle data
diameter_col_index = [6, 10];
angle_col_index = [5, 9];

% Loop through each model
for i = 1:length(models)
    model = models{i}{1}; % Get the current model name
    
    % Construct the file path to the CSV file containing the data for the current model
    csv_filepath = fullfile(data_folder, model, mode, exp_folder, exp_name, 'Characterization', '3DGAC_Matched_Branch_Trait.csv');
    
    % Read the CSV file into a table
    T = readtable(csv_filepath);
    
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
    
    % Calculate MSE and RMSE for diameter estimation
    diameter_mse(i) = mdlr_diameter.MSE;
    diameter_rmse(i) = mdlr_diameter.RMSE;
    
    % Calculate MSE and RMSE for angle estimation
    angle_mse(i) = mdlr_angle.MSE;
    angle_rmse(i) = mdlr_angle.RMSE;

    % Store R-squared values for the current model
    model_names{end+1} = model;
    diameter_linear_r2(end+1) = mdlr_diameter.Rsquared.Ordinary;
    diameter_robust_r2(end+1) = mdblr_diameter.Rsquared.Ordinary;
    angle_linear_r2(end+1) = mdlr_angle.Rsquared.Ordinary;
    angle_robust_r2(end+1) = mdblr_angle.Rsquared.Ordinary;

    % Plot scatterplot for diameter and angle
    figure;
    subplot(1, 2, 1);
    scatter(diameter_data{:, 1}, diameter_data{:, 2});
    hold on;
    plot([min(diameter_data{:, 1}), max(diameter_data{:, 1})], [min(diameter_data{:, 1}), max(diameter_data{:, 1})], 'r--'); % 1-to-1 line
    xlabel('GT');
    ylabel('AppleQSM');
    title(['Diameter Estimation ' model], 'Interpreter', 'none');
    text(max(diameter_data{:, 1}), min(diameter_data{:, 1}), sprintf('MSE: %.4f\nRMSE: %.4f', diameter_mse(i), diameter_rmse(i)), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');

    subplot(1, 2, 2);
    scatter(angle_data{:, 1}, angle_data{:, 2});
    hold on;
    plot([min(angle_data{:, 1}), max(angle_data{:, 1})], [min(angle_data{:, 1}), max(angle_data{:, 1})], 'r--'); % 1-to-1 line
    xlabel('GT');
    ylabel('AppleQSM');
    title(['Angle Estimation ' model], 'Interpreter', 'none');
    text(max(angle_data{:, 1}), min(angle_data{:, 1}), sprintf('MSE: %.4f\nRMSE: %.4f', angle_mse(i), angle_rmse(i)), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');

    set(gcf, 'WindowState', 'maximized');
    saveas(gcf, fullfile(result_folder, [model '.png']));
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