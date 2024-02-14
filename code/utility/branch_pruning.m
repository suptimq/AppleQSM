function cut_branch_indices  = branch_pruning(branch_radius_list, branch_node_list)
    % Initialize variables
    branch_diameters = 2 * branch_radius_list;
    
    % Rule 2: Identify branches with diameter larger than 2cm
    large_branches_indices = find(branch_diameters > 20);
    
    cut_branch_indices = [];
    if isempty(large_branches_indices)
        disp('No branches with diameter larger than 2 cm found.');
        return;
    elseif numel(large_branches_indices) == 1
        disp('Only one branch with diameter larger than 2 cm found. No further pruning is needed.');
        cut_branch_indices(1) = large_branches_indices;
        return;
    end
    
    % Rule 3: Cut the largest branch
    [~, max_diameter_index] = max(branch_diameters(large_branches_indices));
    branch_to_cut = large_branches_indices(max_diameter_index);
    cut_branch_indices(1) = branch_to_cut; % Cutting the largest branch

    % Rule 4: Find the other cut branch by prioritizing the top one
    rest_large_branches = setdiff(large_branches_indices, branch_to_cut);
    max_z_coords = zeros(size(rest_large_branches));
    
    for i = 1:length(rest_large_branches)
        branch_index = rest_large_branches(i);
        branch_node = branch_node_list(branch_index, :);
        max_z_coords(i) = branch_node(3);
    end
    
    [~, top_most_index] = max(max_z_coords);   
    cut_branch_indices(2) = rest_large_branches(top_most_index);
    
    disp(['Branches cut: ', num2str(cut_branch_indices)]);
end