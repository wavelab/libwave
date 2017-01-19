function plot_milestones(fig_index, milestones)
    figure(fig_index);
    hold on;
    plot(milestones(:, 1), milestones(:, 2), 'bo');
end