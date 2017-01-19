function plot_initial_state(figure_index, map, x, X, D)
    figure(figure_index);
    clf; 
    hold on;
    
    plot(map(:, 1), map(:, 2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(x(1, 1), x(2, 1), 'ro--')
    for dd = 1:D
        plot(X(1, dd), X(2, dd), 'b.')
    end
    
    axis equal
    axis([-4 6 -1 7]);
    title('Particle Filter Localization')
    %F(1) = getframe;
end