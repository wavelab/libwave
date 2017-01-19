function plot_steady_state(fig_index, T, x, xss, u, uss)
    figure(fig_index);
    hold on;
    clf;
    
    subplot(4,1,1);
    hold on;
    plot(T, x(1,:), 'b-');
    plot(T, xss(1,:), 'r--');
    title('Angle of Attack')

    subplot(4,1,2);
    hold on;
    plot(T, x(2,:), 'b-');
    plot(T, xss(2,:), 'r--');
    title('Pitch Angle')

    subplot(4,1,3);
    hold on;
    plot(T, x(3,:), 'b-');
    plot(T, xss(3,:), 'r--');
    title('Pitch Rate')

    subplot(4,1,4);
    hold on;
    plot(T(1:end-1), u(:), 'b-');
    plot(T(1:end-1), uss(:), 'r-');
    title('Deflection Angle');
end

