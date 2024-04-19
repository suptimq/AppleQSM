path('refinement', path);

data_folder = 'E:\Result\LLC_02022022\Row13_Branch';
branch_root_folder = 'D:\Data\Apple_Orchard\Lailiang_Cheng\LLC_02022022\Row13_Raw';

tree_folders = dir(fullfile(data_folder, '*_branch'));


% Define cylinder parameters
cylinder_radius = 0.02;
cylinder_length = cylinder_radius / 5;

SHOW = false;

for k = 1:numel(tree_folders)
    
    tree_folder = fullfile(tree_folders(k).folder, tree_folders(k).name);
    exp_folders = dir(fullfile(tree_folder, '*_downsample'));
    
    if k == 1
        data = cell(numel(exp_folders), 1);
    end

    for i = 1:numel(exp_folders)
        exp_folder = fullfile(exp_folders(i).folder, exp_folders(i).name);
        branch_folders = dir(fullfile(exp_folder, '*_Primary'));

        % Check if the cell already exists for this exp_folder
        if k > 1
            results = data{i};
            existing_rows = size(results, 1);
        else
            % Initialize a cell array to store the results for this exp_folder
            results = cell(numel(branch_folders) + 1, 4);
            results(1, :) = {'Tree ID', 'Branch ID', 'AppleQSM Estimated Radius (mm)', 'Cylinder-Fitting Estimated Radius (mm)'};
            existing_rows = 1;
        end

        for j = 1:numel(branch_folders)

            branch_folder = fullfile(branch_folders(j).folder,  branch_folders(j).name);
            parts = split(branch_folders(j).name, '_');
            % Extract section and color
            section = parts{1};
            color = parts{2};
    
            branch_root_pcd_filepath = fullfile(branch_root_folder, tree_folders(k).name, [section '_' color '_Root.ply']);
            skeleton_pcd_filepath = fullfile(branch_folder, 'skel.pcd');
            complete_pcd_filepath = fullfile(branch_folder, 'fine.pcd');
    
            branch_root_pcd = pcread(branch_root_pcd_filepath);
            skeleton_pcd = pcread(skeleton_pcd_filepath);
            complete_pcd = pcread(complete_pcd_filepath);
    
            mean_root_point = mean(branch_root_pcd.Location);
            distances = sqrt(sum((skeleton_pcd.Location - mean_root_point).^2, 2));
            % Sort the distances in ascending order and get the indices
            [sorted_distances, sorted_indices] = sort(distances);
            sorted_skeleton_points = skeleton_pcd.Location(sorted_indices, :);
    
            % Use the 3rd or 4th point to compute the growing direction for
            % robustness
            first_skeleton_index = 10;
            first_skeleton_point = sorted_skeleton_points(first_skeleton_index, :);
            average_vector = sorted_skeleton_points(first_skeleton_index+1, :) - first_skeleton_point;
    
            % Compute the mean of all points in the skeleton point cloud
    %         mean_skeleton_point = mean(skeleton_pcd.Location);
    
    %         % Find the k nearest neighboring points to the mean point
    %         k = 1; % You can adjust this value as needed
    %         [idx, ~] = knnsearch(skeleton_pcd.Location, mean_skeleton_point, 'K', k);
    %         
    %         % Extract the coordinates of the neighboring points
    %         mean_skeleton_point = skeleton_pcd.Location(idx, :);
    %         [~, index] = ismember(mean_skeleton_point, skeleton_pcd.Location, 'rows');
    %         k = 2;
    %         [idx, ~] = knnsearch(skeleton_pcd.Location, mean_skeleton_point, 'K', k);
    % 
    %         neighbor_point = skeleton_pcd.Location(idx(2), :);
    %         average_vector = mean_skeleton_point - neighbor_point;
    
            % Normalize the average vector to have unit length
            cylinder_axis = average_vector / norm(average_vector);
            % Calculate the endpoints of the cylinder along the axis
            cylinder_start_point = first_skeleton_point - cylinder_length/2 * cylinder_axis;
            cylinder_end_point = first_skeleton_point + cylinder_length/2 * cylinder_axis;
    
            % Determine the minimum and maximum coordinates for the ROI
            xmin = min(cylinder_start_point(1), cylinder_end_point(1)) - cylinder_radius;
            xmax = max(cylinder_start_point(1), cylinder_end_point(1)) + cylinder_radius;
            ymin = min(cylinder_start_point(2), cylinder_end_point(2)) - cylinder_radius;
            ymax = max(cylinder_start_point(2), cylinder_end_point(2)) + cylinder_radius;
            zmin = min(cylinder_start_point(3), cylinder_end_point(3)) - cylinder_radius;
            zmax = max(cylinder_start_point(3), cylinder_end_point(3)) + cylinder_radius;
    
            % Define ROI bounding box
            roi_bbox = [xmin, xmax, ymin, ymax, zmin, zmax];
            indices_within_roi = findPointsInROI(complete_pcd, roi_bbox);
            % Select points within the ROI
            complete_pcd_roi = select(complete_pcd, indices_within_roi);
    
            % Cylinder fitting
            maxDistance = 0.005;
            referenceVector = average_vector;
            model = pcfitcylinder(complete_pcd_roi, maxDistance, referenceVector);
            c_radius = model.Radius * 1000;
    
            % CPC optimization
            center = cpc_optimization(complete_pcd_roi.Location);
            distances = sqrt(sum((center-complete_pcd_roi.Location).^2, 2));
            radius = mean(distances)*1000;

            % Store the results in the cell array
            results{existing_rows + j, 1} = tree_folders(k).name;
            results{existing_rows + j, 2} = branch_folders(j).name;
            results{existing_rows + j, 3} = radius;
            results{existing_rows + j, 4} = c_radius;

            % Store the results in the cell array
%             results(end+1, :) = {branch_folder, radius, c_radius};
    
            if SHOW
                figure;
                ax1 = subplot(1, 2, 1);
                plot3(complete_pcd.Location(:,1), complete_pcd.Location(:,2), complete_pcd.Location(:,3), '.', 'Color', [0.5, 0, 0.5]);
                hold on; axis equal;
                % Visualize skeleton point cloud
                plot3(skeleton_pcd.Location(:,1), skeleton_pcd.Location(:,2), skeleton_pcd.Location(:,3), '.', 'Color', [0.5, 0.5, 0.5]);
                % Visualize mean skeleton point
                scatter3(first_skeleton_point(1), first_skeleton_point(2), first_skeleton_point(3), 100, 'r', 'filled');
                % Visualize ROI bounding box
                plotCube(roi_bbox, 'g', 0.1);
        
                ax2 = subplot(1, 2, 2);
                plot3(complete_pcd_roi.Location(:,1), complete_pcd_roi.Location(:,2), complete_pcd_roi.Location(:,3), '.', 'Color', [0.5, 0, 0.5]);
                hold on; axis equal;
                plot(model);
                plot3(center(1), center(2), center(3), '.', 'Color', 'r', 'MarkerSize', 20);
                title(['CPC/Cylinder-Fitting Radius ' num2str(radius, '%.2f') ' / ' num2str(c_radius, '%.2f') ' mm']);
    
                Link = linkprop([ax1, ax2], {'CameraUpVector', 'CameraPosition', 'CameraTarget'});
                setappdata(gcf, 'StoreTheLink', Link);
            end
        end

        % Store the results for this exp_folder in the data cell
        data{i} = results;
    end
end

% Save all the data after the loop
for i = 1:numel(exp_folders)
    filename = fullfile(data_folder, ['results_' exp_folders(i).name '.csv']);
    writetable(cell2table(data{i}), filename, 'WriteVariableNames', false);
end


% Write the results to a CSV file
% csv_filename = 'estimated_radius.csv';
% csv_filepath = fullfile(data_folder, csv_filename);
% writecell(results, csv_filepath);

% Function to plot a cube given its bounding box
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