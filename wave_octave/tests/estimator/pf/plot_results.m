function plot_results(figure_index, T, x, y, u, mu_S, muP_S)
    figure(figure_index);
    clf;
    hold on;

    % plots
    plot(T, x(2:end), 'b');  % state
    plot(T, y, 'ro');  % measurement
    plot(T, u, 'g--');  % input
    plot(T, mu_S,'r--');  % kf belief
    plot(T, muP_S, 'co--');  % pf belief

    % plot details
    title('State and Estimates')
    try
        legend( ...
            'State', ...
            'Measurement', ...
            'Input', ...
            'Kalman Estimate', ...
            'Particle Estimate' ...
        );
    catch
    end_try_catch
end
