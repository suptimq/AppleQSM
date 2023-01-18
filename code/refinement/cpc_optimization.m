function center = cpc_optimization(surface_pts, initial_skeleton_pts)
    %% reference paper https://ieeexplore.ieee.org/document/8981968
    
    if nargin < 2
        initial_skeleton_pts = median(surface_pts, 1);
    end

    surface_pts = double(surface_pts);
    initial_skeleton_pts = double(initial_skeleton_pts);

    options = optimoptions('fminunc', 'StepTolerance', 1e-9, 'OptimalityTolerance', 1e-9, 'MaxIterations', 1);
    center = initial_skeleton_pts;
    counter = 1;
    prev_fval = 0;

    while counter <= 100
        distance = pdist2(surface_pts, center);
        variance = var(distance, 1);
        mean_ = mean(distance);
        lambda_s  = mean_ / variance * 64;
        f = @(vs)parameterfun(vs, surface_pts, lambda_s);
        [center, fval] = fminunc(f, center, options);                                   % optimization
        if abs(fval-prev_fval) < 1e-9                                                               % check break condition
            disp('==================Find Minimum===================');
            break
        end
        prev_fval = fval;
        counter = counter + 1;
    end
end