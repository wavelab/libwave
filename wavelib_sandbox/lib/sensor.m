function r = range(x1, x2, y1, y2)
    r = sqrt((x2 - x1)^2 + (y2 - y1)^2);
end

function b = bearing(mx, my, x1, x2, x3)
    b = mod(atan2(my - x2, mx - x1) - x3 + pi, 2 * pi) - pi;
end

function [m, ind] = find_closest_feature(map, x)
    % find closest feature in a map based on distance
    ind = 0;
    mind = inf;

    for i = 1:length(map(:, 1))
        d = sqrt( (map(i, 1) - x(1))^2 + (map(i, 2) - x(2))^2);
        if (d < mind)
            mind = d;
            ind = i;
        end
    end

    m = transpose(map(ind, :));
end

function retval = is_feature_inview(f, x, rmax, thmax)
    % checks if a feature is in view
    retval = 0;

	% distance in x and y
    dx = f(1) - x(1);
    dy = f(2) - x(2);

	% calculate range and bearing
    r = sqrt(dx^2 + dy^2);
    th = mod(atan2(dy, dx) - x(3), 2 * pi);
    if (th > pi)
        th = th - 2 * pi;
    end

	% check to see if range and bearing is within view
    if ((r < rmax) && (abs(th) < thmax))
        retval = 1;
	else
        retval = 0;
    end
end
