
function plot_trajectory(fig_index, x_store, carrot_store, t)
    figure(fig_index);
    hold on;

%     subplot(3, 1, 1);

    % plot trajectory
    plot(x_store(1, :), x_store(2, :), 'bo');

    % plot carrot
    plot(carrot_store(1, :), carrot_store(2, :), 'go');
    
    for i = 2:length(t)
        % draw box
        if (mod(i, 200) == 0)
            drawbox(fig_index, x_store(1, i), x_store(2, i), x_store(3, i), 12);
        end
    end

    % plot parameters
    title('Trajectory');
    xlabel('x-position (cm)');
    ylabel('y-position (cm)');
    % axis([4 32 4 12]);
    % axis([20 30 5 15]);
    axis equal;
    
end