data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\row13\characterization\';
experiment_name = 'hc_downsample_iter_7\s1';
filename = 'branch_trait.xlsx';
csv_filepath = fullfile(data_folder, experiment_name, filename);

%% load data
sheetname = 'Adjusted';
all_tree_data = readtable(csv_filepath, 'Sheet', sheetname);

x_label = 'Estimation';
y_label = 'Field Measurement';

intercept = false;
%% linear regression and robust linear linear regression
col_index = [3, 5]; % [7, 6]
subtable = all_tree_data(:, col_index);

x = subtable{:, 1};
y = subtable{:, 2};

mdlr = fitlm(subtable, 'Intercept', intercept);
mdblr = fitlm(subtable,  'Intercept', intercept, 'RobustOpts', 'huber');

bls = mdlr.Coefficients{:, 1};
brob = mdblr.Coefficients{:, 1};

bls_r_squared = mdlr.Rsquared.Ordinary;
brob_r_squared = mdblr.Rsquared.Ordinary;

%% crotch angle
xmin = min(all_tree_data{:, col_index(1)});
xmax = max(all_tree_data{:, col_index(1)});
xinterval = (xmax-xmin) / 10;
ymin = min(all_tree_data{:, col_index(2)});
ymax = max(all_tree_data{:, col_index(2)});
yinterval = (ymax-ymin) / 10;

figure('Name', 'Regression', 'Position', get(0, 'Screensize'))
subplot(1, 2, 1)
scatter(x, y, 20, 'blue', 'filled'); hold on
if intercept
    plot(x, bls(1) + bls(2) * x, 'r');
    plot(x, brob(1) + brob(2) * x, 'g');
    text(xmin+xinterval, ymax-yinterval, ['LR: y=', num2str(bls(2), '%.2f'), 'x+', num2str(bls(1), '%.2f'), ' R^2: ', num2str(bls_r_squared, '%.2f')], 'FontSize', 15);
    text(xmin+xinterval, ymax-1.5*yinterval, ['BLR: y=', num2str(brob(2), '%.2f'), 'x+', num2str(brob(1), '%.2f'), ' R^2: ', num2str(brob_r_squared, '%.2f')], 'FontSize', 15);
else
    plot(x, bls(1) * x, 'r');
    plot(x, brob(1) * x, 'g');
    text(xmin+xinterval, ymax-yinterval, ['LR: y=', num2str(bls(1), '%.2f'), 'x', ' R^2: ', num2str(bls_r_squared, '%.2f')], 'FontSize', 15);
    text(xmin+xinterval, ymax-1.5*yinterval, ['BLR: y=', num2str(brob(1), '%.2f'), 'x', ' R^2: ', num2str(brob_r_squared, '%.2f')], 'FontSize', 15);
end
legend('Data Point', 'LR', 'RANSAC LR', 'Location', 'best');
text(xmax-3*xinterval, ymin+0.5*yinterval, ['RMSE: ', num2str(mdlr.RMSE)], 'color', [1, 0, 0], 'FontSize', 15);
text(xmax-3*xinterval, ymin+1*yinterval, ['BLR-RMSE: ', num2str(mdblr.RMSE)], 'color', [1, 0, 0], 'FontSize', 15);
title('Crotch Angle');
xlabel(x_label); ylabel(y_label); grid on;

%% branch diameter
col_index = [7, 6];
subtable = all_tree_data(:, col_index);

x = subtable{:, 1};
y = subtable{:, 2};

mdlr = fitlm(subtable, 'Intercept', intercept);
mdblr = fitlm(subtable,  'Intercept', intercept, 'RobustOpts', 'huber');

bls = mdlr.Coefficients{:, 1};
brob = mdblr.Coefficients{:, 1};

bls_r_squared = mdlr.Rsquared.Ordinary;
brob_r_squared = mdblr.Rsquared.Ordinary;

xmin = min(all_tree_data{:, col_index(1)});
xmax = max(all_tree_data{:, col_index(1)});
xinterval = (xmax-xmin) / 10;
ymin = min(all_tree_data{:, col_index(2)});
ymax = max(all_tree_data{:, col_index(2)});
yinterval = (ymax-ymin) / 10;

subplot(1, 2, 2)
scatter(x, y, 20, 'blue', 'filled'); hold on
if intercept
    plot(x, bls(1) + bls(2) * x, 'r');
    plot(x, brob(1) + brob(2) * x, 'g');
    text(xmin+xinterval, ymax-yinterval, ['LR: y=', num2str(bls(2), '%.2f'), 'x+', num2str(bls(1), '%.2f'), ' R^2: ', num2str(bls_r_squared, '%.2f')], 'FontSize', 15);
    text(xmin+xinterval, ymax-1.5*yinterval, ['BLR: y=', num2str(brob(2), '%.2f'), 'x+', num2str(brob(1), '%.2f'), ' R^2: ', num2str(brob_r_squared, '%.2f')], 'FontSize', 15);
else
    plot(x, bls(1) * x, 'r');
    plot(x, brob(1) * x, 'g');
    text(xmin+xinterval, ymax-yinterval, ['LR: y=', num2str(bls(1), '%.2f'), 'x', ' R^2: ', num2str(bls_r_squared, '%.2f')], 'FontSize', 15);
    text(xmin+xinterval, ymax-1.5*yinterval, ['BLR: y=', num2str(brob(1), '%.2f'), 'x', ' R^2: ', num2str(brob_r_squared, '%.2f')], 'FontSize', 15);
end
legend('Data Point', 'LR', 'RANSAC LR', 'Location', 'best');
text(xmax-3*xinterval, ymin+0.5*yinterval, ['RMSE: ', num2str(mdlr.RMSE)], 'color', [1, 0, 0], 'FontSize', 15);
text(xmax-3*xinterval, ymin+1*yinterval, ['BLR-RMSE: ', num2str(mdblr.RMSE)], 'color', [1, 0, 0], 'FontSize', 15);
title('Branch Diameter');
xlabel(x_label); ylabel(y_label); grid on;

if intercept
    suffix = 'true';
else
    suffix = 'false';
end

saveas(gcf, fullfile(data_folder, experiment_name, ['branch_huber_' sheetname, '_MATLAB_intercept_' suffix '.png']));