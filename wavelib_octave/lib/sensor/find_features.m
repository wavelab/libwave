function meas_ind = find_features(map, x, rmax, thmax)
    meas_ind = [];
    
    for i = 1:length(map)
        if (is_feature_inview(map(:, i), x, rmax, thmax))
            meas_ind = [meas_ind, i];
        end
    end
end

