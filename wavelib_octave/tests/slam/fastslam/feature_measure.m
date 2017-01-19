function y = feature_measure(map, i, x, Qi)
    delta = gaussian_noise(Qi);
    dx = map(1, i) - x(1);
    dy = map(2, i) - x(2);
    y = [
        sqrt(dx^2 + dy^2);
        mod(atan2(dy, dx) - x(3) + pi, 2 * pi) - pi
    ] + delta;
end