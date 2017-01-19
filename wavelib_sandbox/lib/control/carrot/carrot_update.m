function [carrot_t, wp_index] = carrot_update(x_t, r, waypoints, wp_index)
    wp_start = waypoints(wp_index, :);
    wp_end = waypoints(wp_index + 1, :);

    % calculate new carrot
    carrot_t = calculate_new_carrot(x_t, r, wp_start, wp_end);

    % update waypoint if reached waypoint
    if waypoint_reached(x_t(1:2), transpose(wp_end), 2) == 1
        % update waypoint index
        wp_index = wp_index + 1;
        if wp_index  == length(waypoints)
            wp_index = length(waypoints);
            return;
%             wp_index = 1;
        end

        % recalculate carrot
        new_wp_start = waypoints(wp_index, :);
        new_wp_end = waypoints(wp_index + 1, :);
        carrot_t = calculate_new_carrot(x_t, r, new_wp_start, new_wp_end);
    end

%     % limit carrot x component
%     if carrot_t(1) < min(wp_start(1), wp_end(1))
%         carrot_t(1) = min(wp_start(1), wp_end(1));
%     elseif carrot_t(1) > max(wp_start(1), wp_end(1))
%         carrot_t(1) = max(wp_start(1), wp_end(1));
%     end
%     
%     % limit carrot y component
%     if carrot_t(2) < min(wp_start(2), wp_end(2))
%         carrot_t(2) = min(wp_start(2), wp_end(2));
%     elseif carrot_t(2) > max(wp_start(2), wp_end(2))
%         carrot_t(2) = max(wp_start(2), wp_end(2));
%     end
end