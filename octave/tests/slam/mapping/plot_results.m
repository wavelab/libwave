function plot_results(t, map, x, meas_r, meas_phi, invmod, m)
    figure(1);
    clf; 
    
    % parameters
    [M, N] = size(map);

    % map and vehicle path
    subplot(3, 1, 1);
    hold on;
    image(100 * (1 - map));
    colormap(gray);
    plot(x(2, 1:t), x(1, 1:t),'bx-');
    axis equal; 
    axis([0, N, 0, M]);
    title('True Map');


    % inverse measurement model
    subplot(3, 1, 2);
    hold on;
    image(100 * (invmod));
    colormap(gray);
    plot(x(2, t), x(1, t), 'bx')
    for i = 1:length(meas_r)
        plot( ...
            x(2, t) + meas_r(i) * sin(meas_phi(i) + x(3, t)), ...
            x(1, t) + meas_r(i) * cos(meas_phi(i) + x(3, t)), ...
            'ko' ...
        );
    end
    axis equal;
    axis([0, N, 0, M]);
    title('Measurements and inverse measurement model');

    % belief map
    subplot(3, 1, 3);
    hold on;
    image(100 * (m));
    colormap(gray);
    plot(x(2, max(1, t - 10):t), x(1, max(1,t-10):t), 'bx-');
    axis equal;
    axis([0, N, 0, M]);
    title('Current occupancy grid map');
    
    getframe(gca);
end

