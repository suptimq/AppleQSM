function xyz = downsample(CoordinateAndDistance, b, subsample_num, P, pc)
    Color = pc.Color;
    count = pc.Count;
    Limits = [pc.XLimits, pc.YLimits, pc.ZLimits];
    num = fix(2 ^ (3 * P) / b) + 1;
    bin = cell(num, 2);

    for i = 1:num
        bin{i, 2} = [];
    end

    for i = 1:num
        bin{i, 1} = 0;
    end

    for i = 1:count
        location = fix(CoordinateAndDistance(i, 4) / b) + 1;
        bin{location, 1} = bin{location, 1} + 1;
        bin{location, 2} = [bin{location, 2}, i];
    end

    for i = 1:count
        location = fix(CoordinateAndDistance(i, 4) / b) + 1;
        Density(i, 1) = bin{location, 1} / count;
    end

    non_zero = bin(:, 1)';
    non_zero = cell2mat(non_zero);
    non_zero = (non_zero ~= 0);
    non_zero = sum(non_zero(:));
    subsample_num_bin = fix(subsample_num / non_zero) + 1;
    s = RandStream('mlfg6331_64');

    for i = 1:num

        if (bin{i, 1} <= subsample_num_bin)
            continue
        end

        y = randsample(s, bin{i, 2}, subsample_num_bin);
        bin{i, 2} = y;
    end

    selection = [];

    for i = 1:num
        selection = [selection, bin{i, 2}];
    end

    ss = size(selection);
    ss = ss(2);

    for i = 1:ss
        data(i, :) = CoordinateAndDistance(selection(i), :);
        data(i, 1) = data(i, 1) * (Limits(2) - Limits(1)) + Limits(1);
        data(i, 2) = data(i, 2) * (Limits(4) - Limits(3)) + Limits(3);
        data(i, 3) = data(i, 3) * (Limits(6) - Limits(5)) + Limits(5);
        color(i, :) = Color(selection(i), :);
        intensity(i, :) = Density(selection(i));
    end

    data(:, 4) = [];
    xyz = pointCloud(data, 'Color', color, 'Intensity', intensity);
end
