function [spath, sdist] = backtrack_path(nodes, closed_set, dists, start, finish)
    done = 0;
    cur = finish;
    curC = find(closed_set(:, 1) == finish);
    prev =  closed_set(curC, 2);
    spath = [cur];
    sdist = 0;

    while (~done)
        if (prev == start)
            done = 1;
        end

        cur = prev;
        curC = find(closed_set(:,1) == cur);
        prev = closed_set(curC,2);
        spath = [cur, spath];
        sdist = sdist + dists(spath(1), spath(2));
    end

    spath = [nodes(spath, :)];
end