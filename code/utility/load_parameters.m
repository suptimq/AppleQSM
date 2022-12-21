function parameter = load_parameters(paras, key, value, mode)

    if nargin < 4
        mode = 'read';
    end

    if ~isfield(paras, key) || strcmp(mode, 'update')
        parameter = value;
    else
        parameter = getfield(paras, key);
    end

end
