
function wp_reached = waypoint_reached(position, waypoint, threshold)
    wp_reached = 0;

    if dist_between_points(position, waypoint) < threshold
        wp_reached = 1;
        return;
    else
        wp_reached = 0;
        return;
    end
end