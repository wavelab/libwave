function [spath, sdist] = astar(nodes, edges, start, finish)
    [spath, sdist] = shortest_path(1, nodes, edges, start, finish);
end
