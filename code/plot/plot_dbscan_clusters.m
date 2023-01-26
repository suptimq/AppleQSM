%
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPML110
% Project Title: Implementation of DBSCAN Clustering in MATLAB
% Publisher: Yarpiz (www.yarpiz.com)
%
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
%
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

function plot_dbscan_clusters(X, IDX)

    k = double(max(IDX));

    Colors = {'red', 'blue', 'yellow', 'green', 'cyan', 'magenta'};
    Styles = {'.'};

    for i = 1:k
        Xi = X(IDX == i, :);

        if i ~= -1
            Style = Styles{rem(i, length(Styles)) + 1};
            MarkerSize = 30;
            Color = Colors{rem(i, length(Colors)) + 1};
            % Legends{end + 1} = ['c #' num2str(i + 1)];
            % Legends{end + 1} = ['Node #' num2str(i + 1)];
        end

        if ~isempty(Xi)

            [minpt, index] = min(Xi(:, 2));
            plot3(Xi(:, 1), Xi(:, 2), Xi(:, 3), Style, 'MarkerSize', MarkerSize, 'Color', Color, 'MarkerFaceColor', Color);
            hold on

        end

    end

    % legend(Legends);
    % legend('Location', 'NorthEastOutside');
    hold on;
    axis equal;
end
