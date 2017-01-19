function [spath, sdist] = shortest_path(mode, nodes, edges, start, finish)
    % check mode
    if (mode == 1)
        % A-star
        disp('Running A-star mode');
    elseif (mode == 2)
        % Dijkstra
        disp('Running Dijkstra mode');
    else
        disp('Invalid mode!');
        return;
    end

    % setup
    done = 0;
    dists = calculate_edge_distances(nodes, edges);
    dmax = calculate_max_edge_distance(nodes);
    open_set = [start, 0, dmax, 0];  % (node, backtrack, lbound cost, curr cost)
    closed_set = [];  % (node, backtrack, lbound cost, curr cost)

    while (~done)
        % Find best node in open set
        [val, best] = min(open_set(:,3));
        bestnode = open_set(best,:);

        % Move best to closed set
        closed_set = [closed_set; bestnode];

        % Check end condition
        if (isempty(open_set))
            disp('No solution!');
            return;
        elseif (bestnode(1) == finish)
            done = 1;
            continue;
        end

        % Get all neighbours of best node
        neigh = find(edges(bestnode(1),:) == 1);

        % Process each neighbour
        for i = 1:length(neigh)
            % if neighbour is in closed set, skip
            found = find(closed_set(:,1) == neigh(i), 1);
            if (length(found) == 1)
                continue;
            end
            dx = nodes(neigh(i), 1) - nodes(finish, 1);
            dy = nodes(neigh(i), 2) - nodes(finish, 2);
            dtogo = sqrt(double(dx^2 + dy^2));
            dcur = bestnode(4) + dists(bestnode(1), neigh(i));
            found = find(open_set(:,1) == neigh(i),1);

            % if neighbour is not in open set, add it
            if (isempty(found))
                if (mode == 1)
                    % Astar
                    open_set = [open_set; neigh(i) bestnode(1) dtogo+dcur dcur];
                elseif (mode == 2)
                    % Dijkstra
                    open_set = [open_set; neigh(i) bestnode(1) dcur dcur];
                end

            % if neighbour is in open set, check if new route is better
            else
                if (dcur < open_set(found, 4))
                    if (mode == 1)
                        % Astar
                        open_set(found, :) = [neigh(i) bestnode(1) dtogo+dcur dcur];
                    elseif (mode == 2)
                        % Dijkstra
                        open_set(found, :) = [neigh(i) bestnode(1) dcur dcur];
                    end
                end
            end
        end

        % remove best node from open set
        open_set = open_set([1:(best - 1), (best + 1):end], :);

        % plot active nodes for this step
        % plot_active_nodes(1, nodes, bestnode, neigh, closed_set);
    end

    % find and plot final path through back tracing
    [spath, sdist] = backtrack_path(nodes, closed_set, dists, start, finish);
end
