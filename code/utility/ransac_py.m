function [vector, inliers, outliers] = ransac_py(pts, model, min_samples, residual_threshold, max_trials)
    % call the Python script to run 3D RANSAC line fitting

    [vector, inliers, outliers] = pyrunfile("ransac_.py", ["vector", "inliers", "outliers"], ...
                                            value = py.numpy.array(pts), model = py.str(model), min_samples = py.int(min_samples), ...
                                            residual_threshold = py.float(residual_threshold), max_trials = py.int(max_trials));
    if py.NoneType == vector
        vector = nan; inliers = nan; outliers = nan;
    else
        vector = double(vector); inliers = double(inliers); outliers = double(outliers);
    end
end
