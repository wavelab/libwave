source('../lib/ga.m');

function result = cmp(x, y)
    if (x > y)
        result = 1;
    else
        result = 0;
    end
end

population = ga_generate_popoulation(5, 10);
scores = rand(1, 10)

% parents = ga_tournament_selection(population, scores, 2, @cmp)
% ga_point_mutation([0, 0, 0, 0], 1.0);
% [child_1, child_2] = ga_point_crossover([0, 0, 0, 0], [1, 1, 1, 1], 1.0)
