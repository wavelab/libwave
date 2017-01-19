function edges = connect_edges(nodes, nb_edges)
    n = length(nodes);
    edges = zeros(n, n);
    dists = zeros * ones(n, n);

    for i = 1:n
        % calculate distances between nodes
        for j = 1:n
            if (i ~= j)
                dx = nodes(j, 1) - nodes(i, 1);
                dy = nodes(j, 2) - nodes(i, 2);
                d(j) = sqrt(dx^2 + dy^2);
            end
        end

        % sort calculated distances between nodes
        [d2, indicies] = sort(d);

        % create edges to p closest nodes
        for j = 1:min(nb_edges, length(indicies))
            if (i ~= indicies(j))
                edges(i, indicies(j)) = 1;
                edges(indicies(j), i) = 1;
            end
        end
    end
end