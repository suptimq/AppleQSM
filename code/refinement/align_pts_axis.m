function [x, y, bls, A, B] = align_pts_axis(surface_pts, d1, d2, grow_info)
    %% rotate the point cloud along d3 axis
    % % e.g., if d1 = 1 and d2 = 2, the rota

    grow_axis = grow_info.dimension;
    x = surface_pts(:, d1);
    y = surface_pts(:, d2);
    bls = regress(y, [ones(length(x), 1), x]);

    X = atan(bls(2)); % -pi/2~pi/2

    if grow_axis == 2

        if d1 == 1 && d2 == 2 % align with Y axis

            if X < 0
                X =- (pi / 2 + X);
            else
                X = pi / 2 - X;
            end

        else
            X = -X;
        end

    end

    if grow_axis == 1 && d1 == 1 && d2 == 2 % align with X axis
        X = -X;
    end

    % rotate along Z, Y, and X axis
    if (d1 == 1 && d2 == 2) || (d1 == 2 && d2 == 1)
        A = [cos(X) -sin(X) 0 0; sin(X) cos(X) 0 0; 0 0 1 0; 0 0 0 1];
        B = [cos(X) sin(X) 0 0; -sin(X) cos(X) 0 0; 0 0 1 0; 0 0 0 1];
    elseif (d1 == 1 && d2 == 3) || (d1 == 3 && d2 == 1)
        A = [cos(X) 0 sin(X) 0; 0 1 0 0; -sin(X) 0 cos(X) 0; 0 0 0 1];
        B = [cos(X) 0 -sin(X) 0; 0 1 0 0; sin(X) 0 cos(X) 0; 0 0 0 1];
    elseif (d1 == 2 && d2 == 3) || (d1 == 3 && d2 == 2)
        A = [1 0 0 0; 0 cos(X) -sin(X) 0; 0 sin(X) cos(X) 0; 0 0 0 1];
        B = [1 0 0 0; 0 cos(X) sin(X) 0; 0 -sin(X) cos(X) 0; 0 0 0 1];
    end

end
