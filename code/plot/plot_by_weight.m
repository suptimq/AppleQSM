function [] = plot_by_weight(spls, density, show_colormap)

    if nargin < 3
        show_colormap = true;
    end

    % define color of each point based on z value
    [~, ia, ic] = unique(abs(density));
    colors = jet(size(ia, 1));
    colorData = colors(ic, :);

    % define size gradient based on z value
    dotSize = linspace(50, 150, numel(density));
    sizeData = dotSize(ic);
    % plot 3D points
    if size(spls, 2) == 3
        set(gcf, 'color', 'white')
        scatter3(spls(:, 1), spls(:, 2), spls(:, 3), sizeData, colorData, 'filled', 'o', 'MarkerEdgeColor', 'k', ...
            'MarkerFaceAlpha', 0.5);
        xlabel('x')
        ylabel('y')
        zlabel('z')
        axis equal
        % Choose your viewing angle:
        view(0, 180) % (x,z) view
        view(2) % (x,y) view
        view(3) % (x,y,z) view
    elseif size(spls, 2) == 2
        set(gcf, 'color', 'white')
        scatter(spls(:, 1), spls(:, 2), sizeData, colorData, 'filled', 'o', 'MarkerEdgeColor', 'k', ...
            'MarkerFaceAlpha', 0.5);
        xlabel('Distacne to trunk')
        ylabel('Radius')
    end

    grid on
    box on

    if show_colormap
        colormap(colors)
        colorbar
    end
