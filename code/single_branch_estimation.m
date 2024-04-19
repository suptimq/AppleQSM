path('refinement', path);
path('utility', path);
path('confidence', path);

data_folder = 'E:\Result\LLC_02022022\Row13_Branch';
branch_root_folder = 'D:\Data\Apple_Orchard\Lailiang_Cheng\LLC_02022022\Row13_Raw';
branch_gt_filepath = 'D:\Data\Apple_Orchard\Lailiang_Cheng\Branch_GT.csv';

branch_gt_table = readtable(branch_gt_filepath);

tree_folders = dir(fullfile(data_folder, '*_branch'));

% Define CPC parameters
cpc_num_points_threshold = 40;
cylinder_radius = 0.02;
maximum_length = cylinder_radius / 5;

SHOW = false;

for k = 1:numel(tree_folders)

    tree_filename = tree_folders(k).name;
    parts = split(tree_filename, '_');
    tree_idx = parts{1};
    
    tree_folder = fullfile(tree_folders(k).folder, tree_filename);
    exp_folders = dir(fullfile(tree_folder, '*_downsample'));
    
    if k == 1
        data = cell(numel(exp_folders), 1);
    end

    parfor i = 1:numel(exp_folders)
        exp_folder = fullfile(exp_folders(i).folder, exp_folders(i).name);
        branch_folders = dir(fullfile(exp_folder, '*_Primary'));

        % Check if the cell already exists for this exp_folder
        if k > 1
            results = data{i};
            existing_rows = size(results, 1);
        else
            % Initialize a cell array to store the results for this exp_folder
            results = cell(numel(branch_folders) + 1, 5);
            results(1, :) = {'Tree ID', 'Branch ID', 'AppleQSM Estimated Diameter (mm)', 'Cylinder-Fitting Estimated Diameter (mm)', 'GT Diameter (mm)'};
            existing_rows = 1;
        end

        for j = 1:numel(branch_folders)

            branch_folder = fullfile(branch_folders(j).folder,  branch_folders(j).name);
            parts = split(branch_folders(j).name, '_');
            % Extract section and color
            section_idx = parts{1};
            color = parts{2};

            branch_root_pcd_filepath = fullfile(branch_root_folder, tree_filename, [section_idx '_' color '_Root.ply']);
            skeleton_pcd_filepath = fullfile(branch_folder, 'skel.pcd');
            complete_pcd_filepath = fullfile(branch_folder, 'fine.pcd');
    
            branch_root_pcd = pcread(branch_root_pcd_filepath);
            skeleton_pcd = pcread(skeleton_pcd_filepath);
            complete_pcd = pcread(complete_pcd_filepath);
            complete_pcd_location = complete_pcd.Location;
            mean_root_point = mean(branch_root_pcd.Location);

            %% rotate the point cloud so that the center vector is approximately parallel to XY plane
            % find branch grow information
            grow_info = find_grow_direction(complete_pcd, maximum_length);

            % 1st rotation along Z axis and 2nd rotation along either X or Y axis
            % if the branch grows more along X axis
            [x, y, bls, A, B] = align_pts_axis(complete_pcd_location, 1, 2, grow_info);
            tf_surface_pts1 = transformPoint3d(complete_pcd_location, A);

            if grow_info.dimension == 1
                [x2, y2, bls2, A2, B2] = align_pts_axis(tf_surface_pts1, 1, 3, grow_info);
                label = 'x-axis';
            else
                [x2, y2, bls2, A2, B2] = align_pts_axis(tf_surface_pts1, 2, 3, grow_info);
                label = 'y-axis';
            end

            % transform points and find the grow direction (either X or Y axis)
            tf_surface_pts = transformPoint3d(tf_surface_pts1, A2);
            surface_pc = pointCloud(tf_surface_pts);

            % find the transformed branch grow information
            grow_info = find_grow_direction(surface_pc, maximum_length);
            grow_info.maximum_length = maximum_length;

            [cpc_optimized_radius, cpc_optimized_center, cpc_optimized_confidence, segment_inliers] = segment_and_cpc(tf_surface_pts, grow_info, cpc_num_points_threshold);
            
            inlier_spls = cpc_optimized_center(segment_inliers == 1, :);
            outlier_spls = cpc_optimized_center(segment_inliers ~= 1, :);
            inverse_tf = B * B2;
            ori_cpc_optimized_center = transformPoint3d(inlier_spls, inverse_tf);

            distances = sqrt(sum((ori_cpc_optimized_center - mean_root_point).^2, 2));
            % Sort the distances in ascending order and get the indices
            [~, sorted_indices] = sort(distances);
            sorted_optimized_center = ori_cpc_optimized_center(sorted_indices, :);
            sorted_optimized_radius = cpc_optimized_radius(sorted_indices);

            radius = mean(sorted_optimized_radius(1:4), 'omitnan') * 1000;

            %% Sort skeleton points based on their distance to the root point
            distances = sqrt(sum((skeleton_pcd.Location - mean_root_point).^2, 2));
            % Sort the distances in ascending order and get the indices
            [sorted_distances, sorted_indices] = sort(distances);
            sorted_skeleton_points = skeleton_pcd.Location(sorted_indices, :);

            % Use the 3rd or 4th point to compute the growing direction for
            % robustness
            first_skeleton_index = 10;
            first_skeleton_point = sorted_skeleton_points(first_skeleton_index, :);
            average_vector = sorted_skeleton_points(first_skeleton_index+1, :) - first_skeleton_point;
    
            % Normalize the average vector to have unit length
            cylinder_axis = average_vector / norm(average_vector);
            % Calculate the endpoints of the cylinder along the axis
            cylinder_start_point = first_skeleton_point - maximum_length/2 * cylinder_axis;
            cylinder_end_point = first_skeleton_point + maximum_length/2 * cylinder_axis;
    
            % Determine the minimum and maximum coordinates for the ROI
            xmin = min(cylinder_start_point(1), cylinder_end_point(1)) - cylinder_radius/2;
            xmax = max(cylinder_start_point(1), cylinder_end_point(1)) + cylinder_radius/2;
            ymin = min(cylinder_start_point(2), cylinder_end_point(2)) - cylinder_radius/2;
            ymax = max(cylinder_start_point(2), cylinder_end_point(2)) + cylinder_radius/2;
            zmin = min(cylinder_start_point(3), cylinder_end_point(3)) - cylinder_radius;
            zmax = max(cylinder_start_point(3), cylinder_end_point(3)) + cylinder_radius;
    
            % Define ROI bounding box
            roi_bbox = [xmin, xmax, ymin, ymax, zmin, zmax];
            indices_within_roi = findPointsInROI(complete_pcd, roi_bbox);
            % Select points within the ROI
            complete_pcd_roi = select(complete_pcd, indices_within_roi);
    
            %% Cylinder fitting
            maxDistance = 0.005;
            referenceVector = average_vector;
            model = pcfitcylinder(complete_pcd_roi, maxDistance, referenceVector);
            c_radius = model.Radius * 1000;

            idx = find(branch_gt_table.Tree_Index == str2double(tree_idx(5:end)) & branch_gt_table.Section_Index == str2double(section_idx(8:end)) & strcmp(branch_gt_table.Color, color));
            if isempty(idx)
                error('Corresponding row not found for Tree Index %s, Section Index %s, Color %s', tree_idx, section_idx, color);
            end

            %% Store the results in the cell array
            results{existing_rows + j, 1} = tree_idx;
            results{existing_rows + j, 2} = branch_folders(j).name;
            results{existing_rows + j, 3} = radius*2;
            results{existing_rows + j, 4} = c_radius*2;
            results{existing_rows + j, 5} = branch_gt_table.Primary_Branch_Diameter_mm(idx);

            %% Plot
            if SHOW
                figure;
                ax1 = subplot(1, 2, 1);
                plot3(complete_pcd.Location(:,1), complete_pcd.Location(:,2), complete_pcd.Location(:,3), '.', 'Color', [0.5, 0, 0.5]);
                hold on; axis equal;
                % Visualize skeleton point cloud
                plot3(skeleton_pcd.Location(:,1), skeleton_pcd.Location(:,2), skeleton_pcd.Location(:,3), '.', 'Color', [0.5, 0.5, 0.5], 'MarkerSize', 15);
                % Visualize mean skeleton point
                scatter3(first_skeleton_point(1), first_skeleton_point(2), first_skeleton_point(3), 100, 'r', 'filled');
                % Visualize ROI bounding box
                plotCube(roi_bbox, 'g', 0.1);
        
                ax2 = subplot(1, 2, 2);
                plot3(complete_pcd_roi.Location(:,1), complete_pcd_roi.Location(:,2), complete_pcd_roi.Location(:,3), '.', 'Color', [0.5, 0, 0.5]);
                plot3(complete_pcd.Location(:,1), complete_pcd.Location(:,2), complete_pcd.Location(:,3), '.', 'Color', [0.5, 0, 0.5]);
                hold on; axis equal;
                plot(model);
                plot3(ori_cpc_optimized_center(:, 1), ori_cpc_optimized_center(:, 2), ori_cpc_optimized_center(:, 3), '.', 'Color', 'r', 'MarkerSize', 20);
                title(['CPC/Cylinder-Fitting Radius ' num2str(radius, '%.2f') ' / ' num2str(c_radius, '%.2f') ' mm']);
    
                Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget'});
                setappdata(gcf, 'StoreTheLink', Link);
            end
        end

        % Store the results for this exp_folder in the data cell
        data{i} = results;
    end
end

%% Save all the data after the loop
% for i = 1:numel(exp_folders)
%     filename = fullfile(data_folder, ['results_' exp_folders(i).name '.csv']);
%     writetable(cell2table(data{i}), filename, 'WriteVariableNames', false);
% end

threshold = 10;
intercept = false;
diameter_col_index = [3, 5];
diameter_mae = zeros(length(exp_folders), 1);
diameter_rmse = zeros(length(exp_folders), 1);

for i = 1:numel(exp_folders)
    filename = fullfile(data_folder, ['results_' exp_folders(i).name '.csv']);
    
    T = readtable(filename);
    % Extract diameter data from the table
    diameter_data = T(:, diameter_col_index);

    % Fit linear models to the diameter data with and without robust fitting
    mdlr_diameter = fitlm(diameter_data, 'Intercept', intercept); % Linear model
    mdblr_diameter = fitlm(diameter_data, 'Intercept', intercept, 'RobustOpts', 'huber'); % Linear model with robust fitting

    % Identify inliers based on the linear model residuals
    residuals = mdlr_diameter.Residuals.Raw;
    inliers_indices = find(abs(residuals) <= threshold); % Define your threshold for inliers

    % Calculate MSE and RMSE for diameter estimation
    diameter_mae(i) = mae(diameter_data{:, 1}, diameter_data{:, 2});
    diameter_rmse(i) = mdlr_diameter.RMSE;

    figure;
%     subplot(1, 2, 1);
    plot(diameter_data{:, 1}, diameter_data{:, 2}, '.b', 'MarkerSize', 25);
    hold on;
    plot([min(diameter_data{:, 1}), max(diameter_data{:, 1})], [min(diameter_data{:, 1}), max(diameter_data{:, 1})], 'r--'); % 1-to-1 line
    xlabel('AppleQSM'); 
    ylabel('GT');
    title(['Diameter Estimation ' exp_folders(i).name], 'Interpreter', 'none', 'FontSize', 20);
    % Add text for MSE and RMSE
    textX = max(diameter_data{:, 1});
    textY = min(diameter_data{:, 1});
    text(textX, textY, sprintf('MAE: %.4f\nRMSE: %.4f', diameter_mae(i), diameter_rmse(i)), ...
        'FontSize', 20, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');

    set(gcf, 'WindowState', 'maximized');
    saveas(gcf, fullfile(data_folder, [exp_folders(i).name '.png']));
end

%% Function to plot a cube given its bounding box
function plotCube(bbox, color, alpha)
    hold on;
    vertices = [bbox(1), bbox(3), bbox(5);
                bbox(2), bbox(3), bbox(5);
                bbox(2), bbox(3), bbox(6);
                bbox(1), bbox(3), bbox(6);
                bbox(1), bbox(4), bbox(5);
                bbox(2), bbox(4), bbox(5);
                bbox(2), bbox(4), bbox(6);
                bbox(1), bbox(4), bbox(6)];
    faces = [1, 2, 3, 4;
             5, 6, 7, 8;
             1, 2, 6, 5;
             2, 3, 7, 6;
             3, 4, 8, 7;
             4, 1, 5, 8];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, 'FaceAlpha', alpha, 'EdgeColor', 'none');
end