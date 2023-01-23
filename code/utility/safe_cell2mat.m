function A = safe_cell2mat(C)
    %% make sure the length of each cell equal

    rows = cellfun(@numel, C);
    cols = size(C, 2);
    A = zeros(max(rows), cols);

    for k = 1:cols
        A(1:rows(k), k) = C{k};
    end

end
