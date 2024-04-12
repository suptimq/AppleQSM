function [uniquePointCloud] = unique_pcd(pcd)
    % Check if input is a pointCloud object
    if ~isa(pcd, 'pointCloud')
        error('Input must be a pointCloud object.');
    end
    
    % Get points and colors from the point cloud object
    points = pcd.Location;
    colors = pcd.Color;
    
    % Remove duplicate rows
    [~, uniqueIndices, ~] = unique(points, 'rows');
    
    % Extract unique points and colors
    uniquePoints = points(uniqueIndices, :);
    if ~isempty(colors)
        uniqueColors = colors(uniqueIndices, :);
        % Create a new pointCloud object with unique points and colors
        uniquePointCloud = pointCloud(uniquePoints, 'Color', uniqueColors);
    else
        uniquePointCloud = pointCloud(uniquePoints);
    end
    
end
