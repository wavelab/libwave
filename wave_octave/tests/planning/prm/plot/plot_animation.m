function plot_animation(fig_index, x_store, carrot_store, t)
    figure(fig_index);
    hold on;

    for i = 1:length(t)
        % plot trajectory
        plot(x_store(1, i), x_store(2, i), 'bo');

        % plot carrot
        plot(carrot_store(1, i), carrot_store(2, i), 'ro');

        % draw box
        if (mod(i, 5) == 0)
            drawbox(fig_index, x_store(1, i), x_store(2, i), x_store(3, i), 0.3);
        end

        % plot parameters
        axis equal;
        drawnow;
    end
end
