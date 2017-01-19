function plot_nlls_result(subplot_index, map, x0, meas_ind, mu_s, max_set, max_set_length)
   	subplot(2, 2, subplot_index);
    hold on;
    
    plot(map(:,1), map (:,2), 'bo');
    plot(map(meas_ind, 1), map(meas_ind, 2), 'ro');
    plot(x0(1), x0(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    plot(mu_s(1,:), mu_s(2,:), 'mx-', 'LineWidth', 2, 'MarkerSize', 6);

    for n = 1:max_set_length
        cur_pt = map(meas_ind(max_set(n)), :);
        plot(cur_pt(1), cur_pt(2), 'gx');    
    end
    
    title('RANSAC Results');
    legend( ...
        'All Features', ...
        'Visible Features', ...
        'Robot Position', ...
        'Robot Estimation', ...
        'Inliers' ...
    );
    axis equal;
    axis([0 100 0 100]);
end

