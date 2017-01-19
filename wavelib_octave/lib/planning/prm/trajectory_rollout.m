function trajectory_rollout(T, dt, env, x0, xF, uR, uMin, uMax, sMin, sMax, obsEdges, nO, n_traj, vidObj)
    done = 0;
    t = 0;
    x_cur = x0;  % current position
    x_new = zeros(3);
    
    while ((~done) && (t < T))
        score_step = inf;
        for i = 1:n_traj
            % Constant speed, linear distribution of turn rates
            input = [ ...
                uR(1) / 2 + uMin(1), ...
                uR(2) * (i - 1) / (n_traj - 1) + uMin(2) ...
            ];
            steps = sMax; 

            % Propagate Dynamics
            x = x_cur;
            for j = 2:steps
                x(j, :) = x(j-1, :) + [ ...
                    input(1) * cos(x(j - 1, 3)) * dt, ...
                    input(1) * sin(x(j - 1, 3)) * dt, ...
                    input(2) * dt ...
                ]; 
            end
            keep = inpolygon(x(:, 1), x(:, 2), env(:, 1), env(:, 2));

            if (sum(keep) == steps)
                plot(x(:,1), x(:,2),'g');

                % Score the trajectory
                togo_cur = norm(x(end, 1:2) - xF);
                obs_dist = inf;
                for k = 1:nO
                    obs_dist = min( ...
                        obs_dist, ...
                        Dist2Poly(x(end, 1), ...
                        x(end, 2), ...
                        obsEdges(((k-1)*4)+1:4*k,1), ...
                        obsEdges(((k-1)*4)+1:4*k, 2)) ...
                    );
                end
                
                score_cur = togo_cur - 0.1 * obs_dist;
                if (score_cur < score_step)
                    score_step = score_cur;
                    x_new = x(sMin, :);
                    x_plot = x;
                end

            else
                plot(x(:,1), x(:,2),'r');
            end
        end

        % Check if no progress is made
        if (x_new == x_cur)
            x_new(3) = x_new(3) - 0.1;
        else
            plot(x_plot(:, 1), x_plot(:, 2), 'b');
            plot(x_plot(end, 1), x_plot(end, 2), 'bo');
        end
        
        % Get frame or write to video object
        if (vidObj) 
            writeVideo(vidObj, getframe(gcf));
        else
            getframe(gcf);
        end

        % Check if a path from start to end is found
        if (norm(x_cur(1:2) - xF) < 1)
            done = 1;
        end
        x_cur = x_new;
        t = t + 1;
    end
end

