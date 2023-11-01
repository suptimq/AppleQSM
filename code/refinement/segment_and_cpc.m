function [cpc_optimized_radius, cpc_optimized_center, cpc_optimized_confidence, segment_inliers] = segment_and_cpc(surface_pts, grow_info, cpc_num_points_threshold)
    %% divide the entire branch into small segments and run segment-wise CPC
  
    start = grow_info.start;
    end_ = grow_info.end;
    segment_dimension = grow_info.dimension;
    maximum_length = grow_info.maximum_length;

    cpc_optimized_radius = [];
    cpc_optimized_center = [];
    cpc_optimized_confidence = [];
    segment_inliers = [];

    counter = 0;
    in_between_pts_cell = {};
    while start + maximum_length  < end_
        index = (surface_pts(:, segment_dimension) >= start) & (surface_pts(:, segment_dimension) <= start + maximum_length);
        in_between_pts = surface_pts(index, :);
        if ~isempty(in_between_pts)
            if size(in_between_pts, 1) > cpc_num_points_threshold
                center = cpc_optimization(in_between_pts);
                radius = mean(pdist2(double(center), double(in_between_pts)));
                cpc_optimized_radius = [cpc_optimized_radius, radius];
                cpc_optimized_center = [cpc_optimized_center; center];
            
                counter = counter + 1;
                in_between_pts_cell{counter} = in_between_pts;
            end
        end
        start = start + maximum_length;
    end

    if size(cpc_optimized_center, 1) > 3
        min_samples = 2;
        residual_threshold = maximum_length;
        max_trials = 1e3;
        [segment_vector, segment_inliers, ~] = ransac_py(cpc_optimized_center, '3D_Line', min_samples, residual_threshold, max_trials);
        segment_vector_direction = segment_vector(4:6);
    elseif size(cpc_optimized_center, 1) > 2
        segment_inliers = ones(size(cpc_optimized_center, 1), 1);
        segment_vector_direction = cpc_optimized_center(2, :) - cpc_optimized_center(1, :);
    else
        segment_inliers = ones(1, 1);
        cpc_optimized_confidence = 0.001;
    end
   
    % compute confidence only when there are at least two centers to form a vector
    if size(cpc_optimized_center, 1) > 2
    
        segment_inliers_index = find(segment_inliers == 1);

        for i = 1:length(segment_inliers_index)
        
            index = segment_inliers_index(i);
            center = cpc_optimized_center(index, :);
            radius = cpc_optimized_radius(index);
            tmp_pts = [in_between_pts_cell{index}; center];
            plane = createPlane(center, segment_vector_direction);
            projected_points = projPointOnPlane(tmp_pts, plane);
            [~, ~, transformed_pts] = projection_3d(projected_points);

            center2d = transformed_pts(end, 1:2);
            transformed_pts = transformed_pts(1:end-1, 1:2);
            [~, ratios] = angle_confidence(center2d, radius, transformed_pts);
            ratios = num2cell(ratios);
            [center_ratio, boundary_ratio, inside_ratio, angle_ratio] = deal(ratios{:});
            cpc_optimized_confidence = [cpc_optimized_confidence, angle_ratio];
        end
    
    end

end