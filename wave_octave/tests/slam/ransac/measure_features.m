function [meas_list, meas_raw, meas_ind, meas_count] = measure_features(N, map, x0, rmax, thmax, Q)
    meas_list = zeros(N, 1);
    
    j = 1;
    for i = 1:N
        meas_list(i) = is_feature_inview(map(i, :), x0, rmax, thmax);
        if (meas_list(i))
            d = sqrt(Q) * randn(1, 1);
            
            dx = map(i, 1) - x0(1);
            dy = map(i, 2) - x0(2);
            meas_raw(j) = atan2(dy, dx) - x0(3) + d;
            
            j = j + 1;
        end
    end
    
    meas_ind = find(meas_list);
    meas_count = length(meas_ind);
end

