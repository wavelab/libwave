


function plot_omap(fig_index, map, pos_start, pos_end, dxy)
    figure(fig_index);
    hold on;

    colormap('gray');
    imagesc(1 - map');

    % plot
    plot(pos_start(1), pos_start(2), 'ro', 'MarkerSize',10, 'LineWidth', 3);
    plot(pos_end(1), pos_end(2), 'gx', 'MarkerSize',10, 'LineWidth', 3 );

    % plot parameters
    axis equal
end