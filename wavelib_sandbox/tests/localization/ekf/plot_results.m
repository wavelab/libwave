function plot_results(T, map, mf, x, mu, S, mu_S, mup_S, y, meas)
    for t = 2:length(T)
        figure(1);
        clf;
        hold on;
        axis equal
        axis([-4 6 -1 7])

        plot(map(:,1), map(:,2), 'go', 'MarkerSize',10, 'LineWidth', 2);
        plot(mf(1,t), mf(2,t), 'mx', 'MarkerSize',10, 'LineWidth', 2)
        plot(x(1,1:t), x(2,1:t), 'ro--')

        switch(meas)
        case 1
            plot_circle(x(1, t), x(2, t), y(1, t));
        case 2
            plot( ...
                [x(1, t), x(1, t) + 10 * cos(y(1, t) + x(3, t))], ...
                [x(2, t), x(2, t) + 10 * sin(y(1, t) + x(3, t))], ...
                'c' ...
            );
        case 3
            plot_circle(x(1, t), x(2, t), y(1, t));
            plot( ...
                [x(1, t), x(1, t) + y(1, t) * cos(y(2, t) + x(3, t))], ...
                [x(2, t), x(2, t) + y(1, t) * sin(y(2, t) + x(3, t))], ...
                'c' ...
            );
        end
        plot(mu_S(1, 1:t-1), mu_S(2, 1:t-1), 'bx--');
        plot(mup_S(1, 1:t), mup_S(2, 1:t), 'go--');
        
        title('Range & Bearing Measurements for Localization')
        drawnow;
    end
end

