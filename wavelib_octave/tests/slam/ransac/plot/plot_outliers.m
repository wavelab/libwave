function plot_outliers(subplot_index, map, meas_ind, meas_ind_old, outlier_count)
    subplot(2, 2, subplot_index);
    hold on;
    
    % plot
    plot(map(:,1), map (:,2), 'bo')
    plot(map(meas_ind,1), map (meas_ind,2), 'ro')
    %plot (x0(1), x0(2), 'gx', 'MarkerSize', 10, 'LineWidth',2)
    
    for i = 1:outlier_count
        plot( ...
            [map(meas_ind_old(i),1), map(meas_ind(i),1)], ...
            [map(meas_ind_old(i),2) map(meas_ind(i),2)], ...
            'mx' ...
        );
    end
    
    % plot parameters
    title('Outliers included in dataset');
    legend('All features', 'Visible features', 'Outliers');
    axis equal;
    axis([0, 100, 0, 100]);
end

