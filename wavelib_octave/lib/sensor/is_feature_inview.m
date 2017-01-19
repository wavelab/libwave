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
