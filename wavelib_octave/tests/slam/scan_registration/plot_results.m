function plot_results(t, dx, map, xr, xICP, pp, pp2, qq, invmod_T, meas_r,  meas_phi, m, M, N)
    % Map, scans and vehicle path
    figure(1);
    clf; 
    subplot(2, 2, 1); 
    hold on;
    image(100 * (1-map'));
    colormap(gray);
    plot(xr(1,1:t)/dx,xr(2,1:t)/dx, 'bx-')
    plot(xICP(1, 2:t)/dx, xICP(2, 2:t)/dx, 'go')
    RBI = [
        cos(xr(3, t - 1)), -sin(xr(3, t - 1));
        sin(xr(3, t - 1)), cos(xr(3, t - 1))
    ];
    for i = 1:length(pp2)
        pp2I(i, :) = (RBI* pp2(i,:)'/dx + xr(1:2,t-1)/dx)';
    end
    for i = 1:length(qq)
        qqI(i, :) = (RBI * qq(i, :)' / dx + xr(1:2, t - 1) / dx);
    end
    plot(pp2I(:, 1),pp2I(:, 2),'bx');
    plot(qqI(:,1),qqI(:,2),'rx'); 
    legend('Robot pose', 'Scan reg transform', 'Transformed scan', 'Prior scan')
    title('Map, scans and robot poses')
    axis([0 M 0 N])
    axis equal

    % plot scan registration results
    subplot(2,2,2); 
    hold on;
    plot(qq(:,1), qq(:,2),'bo');
    plot(pp(:,1), pp(:,2),'ro');
    plot(pp2(:,1), pp2(:,2),'go');
    legend('Prior Scan','New scan','Transformed scan')
    title('Scan registration results')
    axis equal
    
    % inverse measurement model
    subplot(2,2,3); 
    hold on;
    image(100*(invmod_T'));
    colormap(gray);
    plot(xr(1,t)/dx,xr(2,t)/dx,'bx')
    for i=1:length(meas_r(:,t))
        plot(xr(1,t)/dx+meas_r(i,t)*cos(meas_phi(i) + xr(3,t)),xr(2,t)/dx+meas_r(i,t)*sin(meas_phi(i)+ xr(3,t)),'ko')
    end
    title('Inverse measurement model')
    axis([0 M 0 N])
    axis equal

    % belief map
    subplot(2,2,4); 
    hold on;
    image(100*(m'));
    colormap(gray);
    plot(xICP(1,max(1,t-10):t)/dx,xICP(2,max(1,t-10):t)/dx,'bx-')
    axis([0 M 0 N])
    %F3(t-1) = getframe;
    title('Current occupancy grid map')
    axis([0 M 0 N])
    axis equal
    
    getframe(gca);
end

