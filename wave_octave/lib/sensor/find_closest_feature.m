function [m, ind] = find_closest_feature(map, x)
    % find closest feature in a map based on distance
    ind = 0;
    mind = inf;

    for i = 1:length(map(:, 1))
        d = sqrt( (map(i, 1) - x(1))^2 + (map(i, 2) - x(2))^2);
        if (d < mind)
            mind = d;
            ind = i;
        end
    end

    m = transpose(map(ind, :));
end
