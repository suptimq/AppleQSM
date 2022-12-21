function [curve, uniform_xyz] = spline_interpolation(pts, M)

    curve = cscvn(pts'); % visualization
    x = pts(:, 1)'; y = pts(:, 2)'; z = pts(:, 3)';

    t = [0, cumsum(sqrt(diff(x) .^ 2 + diff(y) .^ 2 + diff(z) .^ 2))];
    t = t / t(end);
    ti = linspace(0, 1, M);
    [xx, ~] = csaps(t, x);
    [yy, ~] = csaps(t, y);
    [zz, ~] = csaps(t, z);
    newx = fnval(xx, ti);
    newy = fnval(yy, ti);
    newz = fnval(zz, ti);
    uniform_xyz = [newx; newy; newz]';

end
