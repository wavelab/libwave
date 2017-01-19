function plot_trajectory_1d(T, x, y, mu)
	figure(1);
	hold on;

    % plot true state, measurement and belief
    plot(T, x(2:end), 'bo--');
    plot(T, y, 'ro--');
    plot(T, mu, 'gx--');

    % plot details
    try
        title('True state, Measurement and Belief');
        legend('True State', 'Measurement', 'Belief');
        xlabel('t');
        ylabel('x');
    catch
    end_try_catch

	pause;
end
