function plot_trajectory_2d(T, x, y, mu)
	figure(1);
	hold on;
	legend_plotted = 0;

	for t = 2:length(T)
        % plot true state, measurement and belief
        plot(x(1, 2:t), x(2, 2:t), 'bo--');
        plot(y(1, 2:t), y(2, 2:t), 'ro');
        plot(mu(1, 2:t), mu(2, 2:t), 'gx--');

        % plot details
        title('True state, Measurement and Belief');
        xlabel('x');
        ylabel('y');
        if (legend_plotted ~= 1)
            try
                legend('True State', 'Measurement', 'Belief');
            catch
                end_try_catch
                legend_plotted = 1;
            end
        end

		drawnow;
	end

	pause;
end
