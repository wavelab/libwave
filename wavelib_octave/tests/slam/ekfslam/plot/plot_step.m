function [newfeature] = plot_step(fig_index, t, map, xr, mu, mu_S, S, y, newfeature)
    figure(fig_index);
    clf;
    
    subplot(2,1,1); 
    hold on;
    plot_map(map);
    plot_state(xr, t);
    plot_belief(mu_S, t);
   
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos, mu_pos,0.75);
    error_ellipse(S_pos, mu_pos,0.95);
    
    for i = 1:length(map)
        if (~newfeature(i))
            fi = 2 * (i - 1) + 1;
            fj = 2 * i;
            plot( ...
                [xr(1,t), xr(1,t)+y(fi,t)*cos(y(fj,t)+xr(3,t))], ...
                [xr(2,t), xr(2,t)+y(fi,t)*sin(y(fj,t)+xr(3,t))], ...
                'c' ...
            );
            plot(mu(3 + fi), mu(3 + fj), 'gx')
            mu_pos = [mu(3 + fi), mu(3 + fj)];
            S_pos = [
                S(3 + fi, 3 + fi), S(3 + fi, 3 + fj); 
                S(3 + fj, 3 + fi), S(3 + fj, 3 + fj)
            ];
            error_ellipse(S_pos, mu_pos, 0.75);
        end
    end
    axis equal
    axis([-8 8 -5 10])
    title('SLAM with Range & Bearing Measurements')
    
    subplot(2, 1, 2);
    image(10000 * S);
    colormap('gray');
    title('Covariance matrix')
    F(t) = getframe(gcf);
end