function plot_states(fig_index, T, x, u)
    figure(fig_index);
    hold on;
    clf;

    subplot(4, 1, 1);
    hold on;
    plot(T, x(1, :), 'b-');
    title('Angle of Attack')

    subplot(4, 1, 2);
    hold on;
    plot(T, x(2, :), 'b-');
    title('Pitch Angle')

    subplot(4, 1, 3);
    hold on;
    plot(T, x(3, :), 'b-');
    title('Pitch Rate')

    subplot(4, 1, 4);
    hold on;
    plot(T(1:end-1), u(:), 'b-');
    title('Deflection Angle')
end

