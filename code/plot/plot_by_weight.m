function [] = plot_by_weight(spls, density, show_colormap, index)

    if nargin < 3
        show_colormap = true;
    end

    if nargin < 4
        index = 1:size(spls, 1);
    end

    % define color of each point based on z value
    [~, ia, ic] = unique(abs(density));
    colors = jet(size(ia, 1));
    colorData = colors(ic, :);

    % define size gradient based on z value
    dotSize = linspace(50, 150, numel(density));
    sizeData = dotSize(ic);

    spls = spls(index, :);
    sizeData = sizeData(index);
    colorData = colorData(index, :);
    % plot 3D points
    if size(spls, 2) == 3
        scatter3(spls(:, 1), spls(:, 2), spls(:, 3), sizeData, colorData, 'filled', 'o', 'MarkerEdgeColor', 'k', ...
            'MarkerFaceAlpha', 0.5);
    elseif size(spls, 2) == 2
        scatter(spls(:, 1), spls(:, 2), sizeData, colorData, 'filled', 'o', 'MarkerEdgeColor', 'k', ...
            'MarkerFaceAlpha', 0.5);
    end

    if show_colormap
        colormap(colors)
        colorbar
    end
