function new_carrot = calculate_new_carrot(x_t, r, wp_start, wp_end)
    % cross-track point on trajectory
    pt_on_line = closest_point(x_t, [wp_start, wp_end]);

    % calculate unit vector of trajectory
    % allows us to traverse distance r along the trajectory
    v = double(wp_end - wp_start);
    u = v / norm(v);

    % update carrot position
    new_carrot = pt_on_line' + r * u;
    new_carrot = [new_carrot(1); new_carrot(2)];
end