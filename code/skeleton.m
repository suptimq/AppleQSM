%% setting
clear;clc;close all;
path('plot', path);
path('utility', path);
path('subsample', path);
path('skeleton', path);

options.bin_size = 5;
options.num_iteration = 2;
options.subsample_num = 50000;
options.subsample_mode = 1; % Hilbert Curve
options.USING_POINT_RING = GS.USING_POINT_RING;

extension = '.ply';
data_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data'; % folder storing original point cloud
skel_folder = 'D:\Code\Apple_Crop_Potential_Prediction\data\skeleton'; % folder storing extracted skeleton

files = dir([data_folder '\' '*' extension]);

for i = 1
    filename = files(i).name;
    filepath = files(i).folder;
    skeletonization(filepath, skel_folder, filename, options);
end

function [] = skeletonization(data_folder, skel_folder, filename_, options)

    close all;
    
    P.filename = [skel_folder '/' filename_]; % which file we should run on

    %% Step 1: read file (point cloud), and subsample point cloud
    tic
    original_pt = pcread([data_folder '/' filename_]);
    original_pt_location = original_pt.Location;
    original_pt_location_normalized = original_pt_location - mean(original_pt_location, 1);
    original_pt_normalized = pointCloud(original_pt_location_normalized, 'Color', original_pt.Color);
    P.original_pt = original_pt_normalized;
    fprintf('number of points for original dataset: %d pts\n', original_pt_normalized.Count);

    P.subsample_num = options.subsample_num;
    if options.subsample_mode
        P.bin_size = options.bin_size;
        P.num_iteration = options.num_iteration;
        downsample_pt_normalized = Hilbertcurve_method(P.num_iteration, P.bin_size, P.subsample_num, original_pt_normalized);
    else
        ratio = P.subsample_num / original_pt_normalized.Count;
        downsample_pt_normalized = pcdownsample(original_pt_normalized, 'random', ratio); % visualization purpose only!
    end
    fprintf('number of points after downsampling: %d pts\n', downsample_pt_normalized.Count);

    P.pts = double(downsample_pt_normalized.Location);
    P.npts = size(P.pts, 1);
    [P.bbox, P.diameter] = GS.compute_bbox(P.pts);
    fprintf('read point set:\n');
    toc

    %% Step 2: build local 1-ring
    tic
    P.k_knn = GS.compute_k_knn(P.npts);

    if options.USING_POINT_RING
        P.rings = compute_point_point_ring(P.pts, P.k_knn, []);
        P.density = compute_density(P.npts, P.pts);
    else
        P.frings = compute_vertex_face_ring(P.faces);
        P.rings = compute_vertex_ring(P.faces, P.frings);
    end

    fprintf('compute local 1-ring:\n');
    toc

    %% Step 3: contract point cloud by Laplacian
    tic
    [P.cpts, t, initWL, WC, sl] = contraction_by_mesh_laplacian(P, options);
    fprintf('Contraction:\n');
    toc
    
    %% Step 4: point to curve ?C by cluster ROSA2.0
    tic
    P.sample_radius = P.diameter * 0.006;
    P = rosa_lineextract(P, P.sample_radius, 1);
    fprintf('to curve:\n');
    toc

    %% Step 5: post-processing remove nan value
    [valid_rows, valid_cols] = find(~isnan(P.spls));
    valid_rows = unique(valid_rows);
    P.spls = P.spls(valid_rows, :);
    P.spls_density = P.spls_density(valid_rows, :);
    P.spls_adj = P.spls_adj(valid_rows, valid_rows);

    default_filename = sprintf('%s_contract_t(%d)_nn(%d)_WL(%f)_WH(%f)_sl(%f)_skeleton', ...
        P.filename(1:end - 4), t, P.k_knn, initWL, WC, sl);
    output_folder = default_filename;

    %% create folder to save results
    if ~exist(output_folder, 'dir')
        mkdir(output_folder)
    end

    %% save results
    save([default_filename '.mat'], 'P');

    %% show results
    figure('Name', 'Original point cloud and its contraction'); movegui('northeast'); set(gcf, 'color', 'white')
    scatter3(P.pts(:, 1), P.pts(:, 2), P.pts(:, 3), 30, '.', 'MarkerEdgeColor', GS.PC_COLOR); hold on;
    scatter3(P.cpts(:, 1), P.cpts(:, 2), P.cpts(:, 3), 30, '.r');
    axis equal; set(gcf, 'Renderer', 'OpenGL');
    camorbit(0, 0, 'camera'); axis vis3d; view(0, 90); view3d rot;
    saveas(gcf, [output_folder '/' 'contraction']);

    figure('Name', 'Original point cloud and its skeleton'); movegui('center'); set(gcf, 'color', 'white');
    scatter3(P.pts(:, 1), P.pts(:, 2), P.pts(:, 3), 20, '.', 'MarkerEdgeColor', GS.PC_COLOR); hold on;
    plot3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), '.r', 'markersize', 30); hold on;

    axis equal;
    saveas(gcf, [output_folder '/' 'skeleton']);

    %% show skeleton connectivity
    figure('Name', 'Skeleton connectivity')
    set(gcf, 'color', 'white')
    sizep = getoptions(options, 'sizep', 100);
    sizee = getoptions(options, 'sizee', 2);
    colorp = getoptions(options, 'colorp', [1, .8, .8]);
    colore = getoptions(options, 'colore', [1, .0, .0]);
    scatter3(P.spls(:, 1), P.spls(:, 2), P.spls(:, 3), sizep, '.'); hold on; axis equal; axis off;
    title('skeleton connectivity');
    set(gcf, 'Renderer', 'OpenGL');
    plot_connectivity(P.spls, P.spls_adj, sizee, colore);
    axis on;
    saveas(gcf, [output_folder '/' 'skeleton connectivity']);

    %% show weighted skeleton
    figure('Name', 'Weighted skeleton')
    plot_by_weight(P.spls, P.spls_density)
    saveas(gcf, [output_folder '/' 'weighted skeleton']);
end
