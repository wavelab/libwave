function b = bearing(f, x)
    dy = f(2) - x(2);
    dx = f(1) - x(1);
%     b = mod(atan2(dy, dx) - x(3) + pi, 2 * pi) - pi;
    b = atan2(dy, dx) - x(3);
end