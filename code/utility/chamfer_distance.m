function cd = chamfer_distance(pc1, pc2, mode)
    if strcmp(mode, 'deterministic')
        % Ensure deterministic behavior
%         [~, D1] = knnsearch(pc2, pc1, 'K', 1, 'NSMethod', 'exhaustive'); % Exact search
%         [~, D2] = knnsearch(pc1, pc2, 'K', 1, 'NSMethod', 'exhaustive'); % Exact search
        D1 = min(pdist2(pc1, pc2, 'euclidean'), [], 2);
        D2 = min(pdist2(pc2, pc1, 'euclidean'), [], 2);
    else
        % Use KDTree for efficient nearest-neighbor search
        tree1 = KDTreeSearcher(pc2);
        tree2 = KDTreeSearcher(pc1);
    
        % Compute nearest neighbor distances
        [~, D1] = knnsearch(tree1, pc1); % Nearest neighbor from pc1 to pc2
        [~, D2] = knnsearch(tree2, pc2); % Nearest neighbor from pc2 to pc1
    end

    % Compute Chamfer Distance
    cd = mean(D1.^2) + mean(D2.^2);
end