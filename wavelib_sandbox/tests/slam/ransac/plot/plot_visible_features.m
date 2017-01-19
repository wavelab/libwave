function plot_visible_features(subplot_index, map, meas_ind, x0)
    subplot(2, 2, subplot_index);
    hold on;
    
    plot(map(:, 1), map(:, 2), 'bo');
    plot(map(meas_ind, 1), map (meas_ind,2), 'ro');
    plot(x0(1), x0(2), 'gx', 'MarkerSize', 10, 'LineWidth',2);
    
    title('Map of features');
    legend('All features', 'Visible features', 'Robot position');
    axis equal;
    axis([0, 100, 0, 100]);
end

