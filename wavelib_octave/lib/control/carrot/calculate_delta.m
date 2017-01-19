function delta_t = calculate_delta(x_t, carrot_t, delta_max)
    theta = x_t(3);

    % calculate angle between carrot and bicycle
    x = (carrot_t(1) - x_t(1));
    y = (carrot_t(2) - x_t(2));
    angle_of_vec = atan3(y, x);  % returns only +ve angle

    % limit delta_t to pi and -pi only
    delta_t = -(theta - angle_of_vec);
    delta_t = mod(delta_t + pi, 2 * pi) - pi;

    % limit delta_t to steering angle max
    if (delta_t > delta_max)
        delta_t = delta_max;
    elseif (delta_t < -delta_max)
        delta_t = -delta_max;
    end
end
