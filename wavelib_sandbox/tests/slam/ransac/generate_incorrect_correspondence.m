function [outlier_ind, outlier_count, meas_ind, meas_ind_old] = generate_incorrect_correspondence(meas_ind, meas_count, outlier_ratio) 
    outlier_count =  floor(outlier_ratio * meas_count);
    outlier_ind = find(rand(outlier_count, 1) > outlier_ratio);
    meas_ind_old = meas_ind;
    
    for i = 1:outlier_count
        meas_ind(i) = ceil(meas_count * rand(1, 1));
    end
end

