function [meanPoint, V, transformed_pts] = projection_3d(pts)
    %%% project 3D pts to the 2D plane that is orthogonal to its normal

    % Mean of all points
    meanPoint = mean(pts, 1);

    % Center points by subtracting the meanPoint
    centeredPoints = pts - repmat(meanPoint, size(pts, 1), 1);

    % Project 3D data to a plane
    [~, ~, V] = svd(centeredPoints);
    transformed_pts = transformPoint3d(centeredPoints, V');

end