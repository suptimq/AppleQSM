function CPC = branch_refinement(P, kdtree, branch_pts_idx, options)
    %% Branch refinement

    CPC = struct;

    CPC.cpc_optimzed_center = [];
    CPC.cpc_optimized_radius = [];
    CPC.cpc_optimized_confidence = [];

    sphere_radius = options.sphere_radius; % 0.01
    maximum_length = options.maximum_length; % 0.002

    branch_pc = [];
    optimized_spls = [];
    optimized_spls_median = [];
    optimized_radius = [];
    optimized_radius_median = [];
    optimized_confidence = [];
    optimized_confidence_median = [];

    for i = 1:length(branch_pts_idx)

        [cpc_optimized_center, cpc_optimized_radius, cpc_optimized_confidence, cpc_optimized_center_outlier, branch_seg_pc] = cpc_refinement(P, branch_pts_idx(i), kdtree, sphere_radius, maximum_length);

        if size(cpc_optimized_center, 1) > 1
            cpc_optimized_center_median = median(cpc_optimized_center);
        else
            cpc_optimized_center_median = cpc_optimized_center;
        end

        cpc_optimized_radius_median = median(cpc_optimized_radius, 'omitnan');
        cpc_optimized_confidence_median = median(cpc_optimized_confidence, 'omitnan');

        branch_pc = [branch_pc; branch_seg_pc];

        if ~isempty(cpc_optimized_center)
            optimized_spls = [optimized_spls; cpc_optimized_center];
            optimized_radius = [optimized_radius, cpc_optimized_radius];
            optimized_confidence = [optimized_confidence, cpc_optimized_confidence];

            optimized_spls_median = [optimized_spls_median; cpc_optimized_center_median];
            optimized_radius_median = [optimized_radius_median; cpc_optimized_radius_median];
            optimized_confidence_median = [optimized_confidence_median; cpc_optimized_confidence_median];
        end

    end

    CPC.branch_pc = pccat(branch_pc);

    CPC.cpc_optimzed_center_median = optimized_spls_median;
    CPC.cpc_optimized_radius_median = optimized_radius_median;
    CPC.cpc_optimized_confidence_median = optimized_confidence_median;

    CPC.cpc_optimzed_center = optimized_spls;
    CPC.cpc_optimized_radius = optimized_radius;
    CPC.cpc_optimized_confidence = optimized_confidence;

end
