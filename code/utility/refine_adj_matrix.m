function [adj_matrix, adj_idx, weight, inv_weight, distance_weight] = refine_adj_matrix(spls, spls_adj, spls_density, distance_th, mode)
    % This function refines the skeleton connectivity by
    %% Cut-off wrong connection
    % ========================================================
    % 1) for connected points, compare the sum of density - density constraint
    % density(i) + density(j) < threshold --> cut
    % ========================================================
    % 2) for connected points, compare the Euclidean distance - distance constraint
    % distance(i, j) > threshold --> cut


    SHOW_RESULTS = false;
    A = spls_adj;
    A(logical(eye(size(A)))) = 0;
    adj_idx = zeros(2, 1e6);
    weight = zeros(1e6, 1);
    inv_weight = zeros(1e6, 1);
    distance_weight = zeros(1e6, 1);
    counter = 1;

    distance_matrix = squareform(pdist(spls));
    density = spls_density ./ max(spls_density); % normalize (0, 1] does 0 make sense physically?
    inv_density = 1 - density;
    density_th = mean(density) + std(density);

    for i = 1:(size(A, 1) - 1)

        for j = (i + 1):size(A, 2)

            dist = distance_matrix(i, j);
            density_sum = density(i) + density(j);
            inv_density_diff = abs(inv_density(i) - inv_density(j));

            % check distance requirement and density requirement
            switch mode
                case 'normal'
                    refine_flag = A(i, j) > 0;
                case 'distance'
                    refine_flag = A(i, j) > 0 || dist < distance_th;
                case 'density'
                    refine_flag = A(i, j) > 0 && density_sum > density_th;
                case 'both'
                    refine_flag = A(i, j) || (dist < distance_th && density_sum > density_th);
            end

            if refine_flag
                A(i, j) = 1;
                adj_idx(:, counter) = [i; j];
                weight(counter, :) = density_sum;
                distance_weight(counter, :) = dist;
                inv_weight(counter, :) = inv_density(i) + inv_density(j);
                counter = counter + 1;
            else
                A(i, j) = 0;
            end

        end

    end

    adj_matrix = A;
    adj_idx = adj_idx(:, 1:counter - 1);
    weight = weight(1:counter - 1, :);
    inv_weight = inv_weight(1:counter - 1, :);
    distance_weight = distance_weight(1:counter - 1, :);

    if SHOW_RESULTS
        figure('Name', 'Inverse weighted')
        plot_by_weight(spls, inv_density);
        axis equal;
    end

end
