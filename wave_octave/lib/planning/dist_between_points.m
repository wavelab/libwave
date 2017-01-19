function dist = dist_between_points(x, y)
	dx = x(1) - y(1);
	dy = x(2) - y(2);
	dist = sqrt(double(dx^2 + dy^2));
    % diff = x - y;
    % dist = norm(diff, 2);
end