clc; clear; close all;
path('confidence', path);
path('utility', path);
path('plot', path);
path('refinement', path);

data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton'; % folder storing extracted skeleton
output_folder = 'C:\Users\tq42\Documents\ASABE_2022_Journal';
exp_id = 'baseline';
extension = '.ply';

files = dir(fullfile(data_folder, ['tree*' extension]));

file = files(3).name;
[filepath, tree_id, ext] = fileparts(file);
skel_filename_format = '_contract_*_skeleton.mat';
skel_filename = search_skeleton_file(tree_id, skel_folder, skel_filename_format);

skel_filepath = fullfile(skel_folder, exp_id, skel_filename);
load(skel_filepath, 'P'); % P results from skeleton operation

%%  visualization purpose only and show the original point clouds
original_pt_normalized = P.original_pt;
original_pt_normalized_location = original_pt_normalized.Location;
desired_pt = 30000;
ratio = desired_pt / original_pt_normalized.Count;
pt = pcdownsample(original_pt_normalized, 'random', ratio); % visualization purpose only!
pt_location = pt.Location;
pt_color = double(pt.Color) / 255;

%% define ellisoid pruning parameters
xy_radius = 0.032;
z_radius = 0.02;
maximum_length = 0.005; % cross-section thickness

optimized_spls = [];
optimized_spls_no_ransac = [];
optimized_outlier_spls = [];
optimized_radius = [];
optimized_confidence = [];
trunk_pc = [];

colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};

for i = 1:length(P.coarse_main_trunk_pts_index)

    pts1 = P.spls(P.coarse_main_trunk_pts_index(i), :);
    x1 = pts1(1); y1 = pts1(2); z1 = pts1(3);

    neighboring_pts_idx = findPointsInROI(original_pt_normalized, [x1 - xy_radius, x1 + xy_radius, y1 - xy_radius, y1 + xy_radius, z1 - z_radius, z1 + z_radius]);
    neighboring_pc = select(original_pt_normalized, neighboring_pts_idx);
    trunk_pc = [trunk_pc; neighboring_pc];

    if ~isempty(neighboring_pc.Location)
        zmin = neighboring_pc.ZLimits(1);
        zmax = neighboring_pc.ZLimits(2);

        %% run CPC on each small trunk segment
        start = zmin;
        tmp_center = [];
        tmp_radius = [];
        tmp_confidence = [];

        color_counter = 1;
        figure('Name', 'RANSAC')

        while start < zmax

            index = (neighboring_pc.Location(:, 3) >= start) & (neighboring_pc.Location(:, 3) <= start + maximum_length);
            in_between_pts = neighboring_pc.Location(index, :);
            plot3(in_between_pts(:, 1), in_between_pts(:, 2), in_between_pts(:, 3), '.', 'Color', colors{rem(color_counter, length(colors)) + 1}, 'MarkerSize', 10); hold on
            color_counter = color_counter + 1;

            center = cpc_optimization(in_between_pts);
            radius = mean(pdist2(double(center), double(in_between_pts)));

            [nums, ratios] = angle_confidence(center(1:2), radius, neighboring_pc.Location(:, 1:2));
            ratios = num2cell(ratios);
            [center_ratio, boundary_ratio, inside_ratio, angle_ratio] = deal(ratios{:});
            total_covered_angle = 360 * angle_ratio;

            tmp_center = [tmp_center; center];
            tmp_radius = [tmp_radius; radius];
            tmp_confidence = [tmp_confidence; angle_ratio];

            start = start + maximum_length;

%             figure('Name', 'Angle confidence')
%             plot(neighboring_pc.Location(:, 1), neighboring_pc.Location(:, 2), '.r', 'Markersize', 30); hold on
%             viscircles(center(1:2), radius, 'Color', 'y');
%             title([' Ratio: ', num2str(center_ratio, '%.2f'), '/', num2str(inside_ratio, '%.2f'), ' Covered Angle ', num2str(total_covered_angle), '/', num2str(angle_ratio, '%.2f')], ...
%                     'color', [1, 0, 0])
%             xlabel('x-axis'); ylabel('y-axis'); grid on;
        end

        optimized_spls_no_ransac = [optimized_spls_no_ransac; median(tmp_center)];

        % 1st local RANSAC to remove outliers
        min_samples = 3;
        residual_threshold = maximum_length;
        max_trials = 1e3;
        [segment_vector, segment_inliers, segment_outliers] = ransac_py(tmp_center, '3D_Line', min_samples, residual_threshold, max_trials);

        inlier_spls = tmp_center(segment_inliers == 1, :);
        inlier_radius = tmp_radius(segment_inliers == 1, :);
        inlier_confidence = tmp_confidence(segment_inliers == 1, :);
        median_inlier_spls = median(inlier_spls);
        median_inlier_radius = median(inlier_radius);
        median_inlier_confidence = median(inlier_confidence);
        outlier_spls = tmp_center(segment_inliers ~= 1, :);

        optimized_spls = [optimized_spls; median_inlier_spls];
        optimized_outlier_spls = [optimized_outlier_spls; outlier_spls];
        optimized_radius = [optimized_radius; median_inlier_radius];
        optimized_confidence = [optimized_confidence; median_inlier_confidence];

        tmp_trunk_pc = pccat(trunk_pc);
        tmp_trunk_pc_location = tmp_trunk_pc.Location;
        tmp_trunk_pc_color = double(tmp_trunk_pc.Color) / 255;
%         scatter3(tmp_trunk_pc_location(:, 1), tmp_trunk_pc_location(:, 2), tmp_trunk_pc_location(:, 3), 10, tmp_trunk_pc_color, 'filled', 'o'); hold on
        plot3(optimized_spls(:, 1), optimized_spls(:, 2), optimized_spls(:, 3), '.white', 'MarkerSize', 30)
        plot3(inlier_spls(:, 1), inlier_spls(:, 2), inlier_spls(:, 3), '.r', 'MarkerSize', 30);
        plot3(outlier_spls(:, 1), outlier_spls(:, 2), outlier_spls(:, 3), '.b', 'MarkerSize', 30);
        plot3(median_inlier_spls(1), median_inlier_spls(2), median_inlier_spls(3), '.g', 'MarkerSize', 40);
        xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

    end

end

%% 2nd semi-global RANSAC to remove outliers
N = 10;
start = 0;
ransac_inlier_spls = [];
ransac_inlier_radius = [];
ransac_inlier_confidence = [];
ransac_outlier_spls = [];

while start <= size(optimized_spls, 1)

    if start + N <= size(optimized_spls, 1)
        index = start + 1:start + N;
        min_samples = 6;
    else
        index = start + 1:size(optimized_spls, 1);
        min_samples = 3;
    end

    % RANSAC parameters
    residual_threshold = maximum_length;
    max_trials = 1e3;

    % select small segment
    segment = optimized_spls(index, :);
    segment_radius = optimized_radius(index);
    segment_confidence = optimized_confidence(index);

    if size(segment, 1) > min_samples
        [segment_vector, segment_inliers, segment_outliers] = ransac_py(segment, '3D_Line', min_samples, residual_threshold, max_trials);

        ransac_inlier_spls = [ransac_inlier_spls; segment(segment_inliers == 1, :)];
        ransac_inlier_radius = [ransac_inlier_radius; segment_radius(segment_inliers == 1)];
        ransac_inlier_confidence = [ransac_inlier_confidence; segment_confidence(segment_inliers == 1)];
        ransac_outlier_spls = [ransac_outlier_spls; segment(segment_inliers ~= 1, :)];
    else
        ransac_inlier_spls = [ransac_inlier_spls; segment];
        ransac_inlier_radius = [ransac_inlier_radius; segment_radius];
        ransac_inlier_confidence = [ransac_inlier_confidence; segment_confidence];
    end

    start = start + N;
end

%% fit a spline and uniformly sample points from the spline
trunk_pc = pccat(trunk_pc);
trunk_pc_zmin = trunk_pc.ZLimits(1);
trunk_pc_zmax = trunk_pc.ZLimits(2);

M = 100; % #sample
[curve, uniform_xyz] = spline_interpolation(ransac_inlier_spls, M);

%% KNN for density
sampled_radius = [];
sampled_confidence = [];
kdtree_spls = KDTreeSearcher(ransac_inlier_spls);

for i = 1:size(uniform_xyz, 1)
    sampled_spls = uniform_xyz(i, :);
    [index, ~] = knnsearch(kdtree_spls, sampled_spls, 'K', 3);
    mean_sampled_radius = mean(ransac_inlier_radius(index));
    if i > 1 
        prev_sampled_radius = sampled_radius(end);
        if mean_sampled_radius > prev_sampled_radius
            mean_sampled_radius = prev_sampled_radius;
        end
    end
    sampled_radius = [sampled_radius, mean_sampled_radius];
    sampled_confidence = [sampled_confidence, mean(ransac_inlier_confidence(index))];
end

%% plot
figure('Name', 'Optimized skeleton pts')
% ax1 = subplot(1, 3, 1);
% pcshow(pt, 'MarkerSize', 20); hold on
% set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
scatter3(pt_location(:, 1), pt_location(:, 2), pt_location(:, 3), 10, pt_color, 'filled', 'o'); hold on
plot3(P.coarse_main_trunk_pts(:, 1), P.coarse_main_trunk_pts(:, 2), P.coarse_main_trunk_pts(:, 3), '.r', 'MarkerSize', 30)
% title('Coarse main trunk', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal; 

% ax2 = subplot(1, 3, 2);
% pcshow(pt, 'MarkerSize', 20); hold on
% set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
figure;
scatter3(pt_location(:, 1), pt_location(:, 2), pt_location(:, 3), 10, pt_color, 'filled', 'o'); hold on
plot3(optimized_spls(:, 1), optimized_spls(:, 2), optimized_spls(:, 3), '.r', 'MarkerSize', 30)
plot3(optimized_outlier_spls(:, 1), optimized_outlier_spls(:, 2), optimized_outlier_spls(:, 3), '.b', 'MarkerSize', 30)
% title('After 1st RANSAC', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

% ax3 = subplot(1, 3, 3);
% pcshow(pt, 'MarkerSize', 20); hold on
% set(gcf, 'color', 'white'); set(gca, 'color', 'white', 'XColor', 'black', 'YColor', 'black', 'ZColor', 'black');
figure;
scatter3(pt_location(:, 1), pt_location(:, 2), pt_location(:, 3), 10, pt_color, 'filled', 'o'); hold on
plot3(ransac_inlier_spls(:, 1), ransac_inlier_spls(:, 2), ransac_inlier_spls(:, 3), '.r', 'MarkerSize', 30)
plot3(ransac_outlier_spls(:, 1), ransac_outlier_spls(:, 2), ransac_outlier_spls(:, 3), '.b', 'MarkerSize', 30)
% title('After 2nd RANSAC', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

% Link = linkprop([ax1, ax2, ax3], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
% setappdata(gcf, 'StoreTheLink', Link);

figure;
scatter3(pt_location(:, 1), pt_location(:, 2), pt_location(:, 3), 10, pt_color, 'filled', 'o'); hold on
plot3(uniform_xyz(:, 1), uniform_xyz(:, 2), uniform_xyz(:, 3), '.r', 'MarkerSize', 30);
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

figure('Name', 'Spline fitting and uniform sampling')
ax1 = subplot(1, 4, 1);
plot3(ransac_inlier_spls(:, 1), ransac_inlier_spls(:, 2), ransac_inlier_spls(:, 3), '.r', 'MarkerSize', 30); hold on
fnplt(curve, 'b', 2)
% title('Spline', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax2 = subplot(1, 4, 2);
plot_by_weight(ransac_inlier_spls, ransac_inlier_radius / xy_radius, false)
% title('Spline', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax3 = subplot(1, 4, 3);
% pcshow(pt, 'MarkerSize', 20); hold on
plot3(uniform_xyz(:, 1), uniform_xyz(:, 2), uniform_xyz(:, 3), '.r', 'MarkerSize', 30);
% title('Uniform sample', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

ax4 = subplot(1, 4, 4);
plot_by_weight(uniform_xyz, sampled_radius / xy_radius, false)
% title('Uniform sample radius', 'color', [1, 0, 0])
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

% ax4 = subplot(1, 4, 4);
% plot_by_weight(uniform_xyz, sampled_confidence)
% % title('Uniform sample confidence', 'color', [1, 0, 0])
% xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis'); grid on; axis equal;

Link = linkprop([ax1, ax2, ax3, ax4], {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);
