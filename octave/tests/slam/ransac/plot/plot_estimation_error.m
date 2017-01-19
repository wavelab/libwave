function plot_estimation_error(r_size, r_error)
    figure(4);
    clf;
    hold on;

    % plot
    plot(r_size, r_error, 'bo');
    
    % plot parameters
    axis([0 100 0 120]);
    title('Estimation error vs inlier set size');
    ylabel('Norm of estimation error');
    xlabel('Inlier set size');
end

