function dists = calculate_edge_distances(nodes, edges)
    n = length(nodes);
    dists= zeros(n, n);

    for i = 1:n
        for j = i:n
            if (edges(i, j))
                dx = nodes(i, 1) - nodes(j, 1);
                dy = nodes(i, 2) - nodes(j, 2);
                dists(i, j) = sqrt(double(dx^2 + dy^2));
                dists(j, i) = dists(i, j);
            end
        end
    end
end
