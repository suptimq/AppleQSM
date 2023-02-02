data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\';
experiment_name = '.';
filename = 'all_tree_trait.xlsx';
csv_filepath = fullfile(data_folder, experiment_name, filename);

%% load data
all_tree_data = readtable(csv_filepath);

x_label = 'Estimation';
y_label = 'Field Measurement';

%% linear regression and robust linear linear regression
col_index = [2, 7];
subtable = all_tree_data(:, col_index);

x = str2double(subtable{:, 1})*100+60;
y = subtable{:, 2};

new_subtable = table(x, y);

mdlr = fitlm(new_subtable);
mdblr = fitlm(new_subtable, 'RobustOpts', 'on');

bls = mdlr.Coefficients{:, 1};
brob = mdblr.Coefficients{:, 1};

bls_r_squared = mdlr.Rsquared.Ordinary;
brob_r_squared = mdblr.Rsquared.Ordinary;

%% crotch angle
xmin = min(x);
xmax = max(x);
xinterval = (xmax-xmin) / 10;
ymin = min(y);
ymax = max(y);
yinterval = (ymax-ymin) / 10;

figure('Name', 'Regression', 'Position', get(0, 'Screensize'))
subplot(1, 2, 1)
scatter(x, y, 20, 'blue', 'filled'); hold on
plot(x, bls(1) + bls(2) * x, 'r');
plot(x, brob(1) + brob(2) * x, 'g');
legend('Data Point', 'LR', 'RANSAC LR', 'Location', 'best');
text(xmin+xinterval, ymax-yinterval, ['LR: y=', num2str(bls(2), '%.2f'), 'x+', num2str(bls(1), '%.2f'), ' R^2: ', num2str(bls_r_squared, '%.2f')], 'FontSize', 15);
text(xmin+xinterval, ymax-1.5*yinterval, ['BLR: y=', num2str(brob(2), '%.2f'), 'x+', num2str(brob(1), '%.2f'), ' R^2: ', num2str(brob_r_squared, '%.2f')], 'FontSize', 15);
text(xmax-3*xinterval, ymin+0.5*yinterval, ['RMSE: ', num2str(mdlr.RMSE)], 'color', [1, 0, 0], 'FontSize', 15);
text(xmax-3*xinterval, ymin+1*yinterval, ['BLR-RMSE: ', num2str(mdblr.RMSE)], 'color', [1, 0, 0], 'FontSize', 15);
text(xmax-3*xinterval, ymin+1.5*yinterval, ['MSE: ', num2str(mdlr.MSE)], 'color', [1, 0, 0], 'FontSize', 15);
title('Tree Height');
xlabel(x_label); ylabel(y_label); grid on;

%% trunk diameter
col_index = [4, 10];
subtable = all_tree_data(:, col_index);

x = str2double(subtable{:, 1})*2*1000;
y = subtable{:, 2};

new_subtable = table(x, y);

mdlr = fitlm(new_subtable);
mdblr = fitlm(new_subtable, 'RobustOpts', 'on');

bls = mdlr.Coefficients{:, 1};
brob = mdblr.Coefficients{:, 1};

bls_r_squared = mdlr.Rsquared.Ordinary;
brob_r_squared = mdblr.Rsquared.Ordinary;

xmin = min(x);
xmax = max(x);
xinterval = (xmax-xmin) / 10;
ymin = min(y);
ymax = max(y);
yinterval = (ymax-ymin) / 10;

subplot(1, 2, 2)
scatter(x, y, 20, 'blue', 'filled'); hold on
plot(x, bls(1) + bls(2) * x, 'r');
plot(x, brob(1) + brob(2) * x, 'g');
legend('Data Point', 'LR', 'RANSAC LR', 'Location', 'best');
text(xmin+xinterval, ymax-yinterval, ['LR: y=', num2str(bls(2), '%.2f'), 'x+', num2str(bls(1), '%.2f'), ' R^2: ', num2str(bls_r_squared, '%.2f')], 'FontSize', 15);
text(xmin+xinterval, ymax-1.5*yinterval, ['BLR: y=', num2str(brob(2), '%.2f'), 'x+', num2str(brob(1), '%.2f'), ' R^2: ', num2str(brob_r_squared, '%.2f')], 'FontSize', 15);
text(xmax-3*xinterval, ymin+0.5*yinterval, ['RMSE: ', num2str(mdlr.RMSE)], 'color', [1, 0, 0], 'FontSize', 15);
text(xmax-3*xinterval, ymin+1*yinterval, ['BLR-RMSE: ', num2str(mdblr.RMSE)], 'color', [1, 0, 0], 'FontSize', 15);
text(xmax-3*xinterval, ymin+1.5*yinterval, ['MSE: ', num2str(mdlr.MSE)], 'color', [1, 0, 0], 'FontSize', 15);
title('Trunk Diameter');
xlabel(x_label); ylabel(y_label); grid on;

saveas(gcf, fullfile(data_folder, experiment_name, 'tree_MATLAB_1.png'));