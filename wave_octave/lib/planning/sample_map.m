function milestones = sample_map(nb_samples, map, map_min, map_max, pos_start, pos_end)
    map_range = map_max - map_min;
    samples_x = int32(map_range(1) * rand(nb_samples, 1) + map_min(1));
    samples_y = int32(map_range(2) * rand(nb_samples, 1) + map_min(2));
    samples = [samples_x, samples_y];
    
    milestones = [pos_start(1:2); pos_end];
    for i = 1:length(samples)
        sample = samples(i, :);   

        if (map(sample(1), sample(2)) == 0)
            milestones = [milestones; sample];
        end
    end    
end