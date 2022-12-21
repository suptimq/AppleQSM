function angle = compute_angle(x, y)

    if size(x, 2) == 3
        angle = atan2d(norm(cross(x, y)), dot(x, y));
    else
        x1 = x(:, 1); y1 = x(:, 2);
        x2 = y(:, 1); y2 = y(:, 2);
        angle = atan2d(y1, x1) - atan2d(y2, x2);

        minus_sign_angle = angle <= 0;
        angle(minus_sign_angle) = 360 + angle(minus_sign_angle);

    end

end
