function edges = connect_edges_omap(map, nodes, nb_edges)
    n = length(nodes);
    edges = zeros(n, n);
    dists = zeros * ones(n, n);

    for i = 1:n
        % calculate distances between nodes
        for j = 1:n
            if (i ~= j)
                dx = nodes(j, 1) - nodes(i, 1);
                dy = nodes(j, 2) - nodes(i, 2);
                d(j) = sqrt(double((dx^2 + dy^2)));
            end
        end

        % sort calculated distances between nodes
        [d2, indicies] = sort(d);

        % create edges to p closest nodes
        for j = 1:min(nb_edges, length(nodes))
            x0 = nodes(i, 1);
            y0 = nodes(i, 2);
            x1 = nodes(indicies(j), 1);
            y1 = nodes(indicies(j), 2);

            if i == indicies(j)
                continue;
			elseif (edges(i, indicies(j)) == 1 || edges(indicies(j), i) == 1)
                continue;
            end

            % collision detection
            collide = omap_collision(map, x0, y0, x1, y1);
            if collide == 0
                edges(i, indicies(j)) = 1;
                edges(indicies(j), i) = 1;
                % plot([x0, x1], [y0, y1], 'y.-');
                % drawnow;
            else
                edges(i, indicies(j)) = 0;
                edges(indicies(j), i) = 0;
                % plot([x0, x1], [y0, y1], 'r-');
                % drawnow;
            end
        end
    end
end